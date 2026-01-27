import cv2
import numpy as np
import websocket
import threading
import time
import json
import os
import urllib.request
from enum import Enum
import traceback

# --- CONFIGURATION ---
VISION_URL = "http://192.168.0.126/"      
ACTUATOR_WS = "ws://192.168.0.118/ws"    

# Frame parameters (320x240 from your stream)
X_CENTER = 160          
STEER_DEADZONE = 50     # Increased for 320px width
STOP_AREA = 18000       

# Speed Settings - OPTIMIZED FOR AUTONOMOUS
SPD_MAX_AUTO = 180      
SPD_MIN_AUTO = 140      # Increased minimum
SPD_KICKSTART = 220     # Reduced slightly for control
SPD_ALIGN = 150         # Increased for better turning
SPD_SEARCH = 130        # Increased for more torque

os.environ['OPENCV_LOG_LEVEL'] = 'OFF'

class Mode(Enum):
    MANUAL = "MANUAL CONTROL"
    AUTO = "AUTONOMOUS MISSION"

class VideoStream:
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.ret = False
        self.running = True
        self.lock = threading.Lock()
        
        # Test connection
        try:
            import requests
            self.session = requests.Session()
            print(f"✅ Connecting to Vision Node: {url}")
            threading.Thread(target=self.update, daemon=True).start()
        except Exception as e:
            print(f"❌ Vision Initialization Failed: {e}")
            self.running = False

    def update(self):
        import requests
        while self.running:
            try:
                # stream=True keeps the connection open
                resp = self.session.get(self.url, stream=True, timeout=5)
                bytes_data = b''
                
                # Iterate over the stream content in small chunks
                for chunk in resp.iter_content(chunk_size=1024):
                    if not self.running: break
                    bytes_data += chunk
                    
                    # Search for JPEG start/end
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        # --- LATENCY FIX: If there's more data in the buffer, SKIP this frame ---
                        # This ensures we only decode the LATEST frame in the pipe
                        if len(bytes_data) > 2048:
                            continue

                        # Decode only if we have a significant number of bytes
                        if len(jpg) > 100:
                            nparr = np.frombuffer(jpg, dtype=np.uint8)
                            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            if img is not None:
                                with self.lock:
                                    self.frame = img
                                    self.ret = True
            except Exception as e:
                with self.lock: self.ret = False
                print(f"⚠️ Vision Link Jitter: {e}. Retrying...")
                time.sleep(1)

    def read(self):
        with self.lock:
            if self.ret and self.frame is not None:
                return True, self.frame
        return False, np.zeros((240, 320, 3), dtype=np.uint8)

    def stop(self):
        self.running = Falseq

class MotorController:
    """Dedicated motor controller with Thread Safety and Physics"""
    def __init__(self):
        self.ws = None
        self.current_speed = 0
        self.current_direction = "S"
        self.last_command_time = 0
        self.command_queue = []
        self.connected = False
        self.lock = threading.Lock() # 🔒 The Thread Lock
        
        self.connect_actuator()
        
        # Start command processing thread
        threading.Thread(target=self._command_processor, daemon=True).start()
        threading.Thread(target=self.heartbeat, daemon=True).start()
    
    def connect_actuator(self):
        with self.lock: # Ensure connection doesn't happen while sending
            try:
                self.ws = websocket.create_connection(ACTUATOR_WS, timeout=2)
                self.connected = True
                print("✅ Actuator Node Linked")
            except Exception as e:
                print(f"❌ Actuator Offline: {e}")
                self.connected = False
    
    def heartbeat(self):
        while True:
            if self.connected and self.ws:
                try:
                    # 🔒 Protect the heartbeat send
                    with self.lock:
                        self.ws.send(json.dumps({"cmd": "H"}))
                except:
                    self.connected = False
                    self.connect_actuator()
            time.sleep(0.1)
    
    def _send_command(self, direction, speed):
        if not self.connected or not self.ws:
            return False
        
        try:
            # 🔒 Protect the command send
            with self.lock:
                cmd = json.dumps({"cmd": "M", "dir": direction, "spd": speed})
                self.ws.send(cmd)
            
            self.current_direction = direction
            self.current_speed = speed
            self.last_command_time = time.time()
            return True
        except Exception as e:
            # Check for specifically the 10053 error to trigger reconnect
            print(f"Send command error: {e}")
            self.connected = False
            return False
    
    def _command_processor(self):
        while True:
            if self.command_queue:
                # Remove redundant commands: only process the LATEST logic
                # This prevents the "Queue Lag"
                direction, speed, priority = self.command_queue.pop() # Use pop() not pop(0)
                self.command_queue.clear() # Clear the rest of the outdated commands
                
                if self.current_direction == "S" and direction != "S":
                    self._send_command(direction, SPD_KICKSTART)
                    time.sleep(0.12) # Slightly longer kickstart
                    self._send_command(direction, speed)
                else:
                    self._send_command(direction, speed)
            
            time.sleep(0.05) # Reduced to 20Hz (Slower is smoother for ESP32)
    
    def request_move(self, direction, speed, priority=1):
        """Only add to queue if direction actually changed"""
        if direction != self.current_direction:
            self.command_queue.append((direction, speed, priority))

class SmartBrain:
    """Main navigation brain with fixed autonomous logic"""
    def __init__(self):
        self.motor = MotorController()
        self.mode = Mode.MANUAL
        self.target_id = None  
        self.search_dir = "R"
        
        # State tracking
        self.state = "IDLE"
        self.last_state_change = time.time()
        self.marker_lost_time = 0
        self.marker_found_time = 0
        self.last_marker_position = X_CENTER
        self.oscillation_prevention = 0
        
        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        print("✅ Navigation Brain Initialized")
    
    def calculate_scaled_speed(self, area):
        """Calculate speed based on distance to marker"""
        if area >= STOP_AREA:
            return 0
        
        # Linear speed reduction as we get closer
        distance_factor = 1.0 - (area / STOP_AREA)
        speed = SPD_MIN_AUTO + (distance_factor * (SPD_MAX_AUTO - SPD_MIN_AUTO))
        
        # Add oscillation damping
        if self.oscillation_prevention > 0:
            speed = max(SPD_MIN_AUTO, speed * 0.7)
        
        return int(speed)
    
    def detect_markers(self, frame):
        """Detect markers and return their data"""
        if frame is None or frame.size == 0:
            return []
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        markers = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:, 0])
                area = cv2.contourArea(marker_corners)
                
                markers.append({
                    'id': int(marker_id),
                    'center_x': center_x,
                    'area': area,
                    'corners': marker_corners
                })
        
        return markers
    
    def update_state_machine(self, markers, current_time):
        """Main state machine for autonomous navigation"""
        # Check for hazards first
        hazard_markers = [m for m in markers if m['id'] == 0]
        if hazard_markers:
            self.state = "HAZARD"
            self.motor.request_move("S", 0, priority=10)
            return
        
        # If no target selected, stay idle
        if self.target_id is None:
            self.state = "IDLE"
            self.motor.request_move("S", 0)
            return
        
        # Find target marker
        target_markers = [m for m in markers if m['id'] == self.target_id]
        
        if target_markers:
            # Found target marker
            self.marker_found_time = current_time
            self.marker_lost_time = 0
            
            marker = target_markers[0]
            center_x = marker['center_x']
            area = marker['area']
            
            # Update last known position for search direction
            self.last_marker_position = center_x
            if center_x < X_CENTER:
                self.search_dir = "L"
            else:
                self.search_dir = "R"
            
            # Check if we've arrived
            if area > STOP_AREA:
                self.state = "ARRIVED"
                self.motor.request_move("S", 0, priority=5)
                return
            
            # Calculate alignment error
            error = center_x - X_CENTER
            
            # Determine state based on error
            if abs(error) > STEER_DEADZONE:
                if error < 0:
                    self.state = "ALIGN_LEFT"
                    self.motor.request_move("L", SPD_ALIGN, priority=2)
                else:
                    self.state = "ALIGN_RIGHT"
                    self.motor.request_move("R", SPD_ALIGN, priority=2)
                
                # Reset oscillation counter when aligning
                self.oscillation_prevention = max(0, self.oscillation_prevention - 1)
            else:
                self.state = "APPROACHING"
                speed = self.calculate_scaled_speed(area)
                self.motor.request_move("F", speed, priority=1)
                
                # Detect oscillation (rapid L-R switching)
                state_duration = current_time - self.last_state_change
                if state_duration < 0.5 and self.state == "APPROACHING":
                    self.oscillation_prevention += 1
            
            self.last_state_change = current_time
            
        else:
            # Target marker not found
            current_time = time.time()
            
            if self.marker_lost_time == 0:
                self.marker_lost_time = current_time
            
            lost_duration = current_time - self.marker_lost_time
            
            if lost_duration < 1.0:  # Wait 1 second before searching
                self.state = "PAUSED"
                self.motor.request_move("S", 0)
            else:
                self.state = "SEARCHING"
                # Alternate search direction every 3 seconds
                if lost_duration > 6.0:  # If searching for >6 seconds
                    self.search_dir = "R" if self.search_dir == "L" else "L"
                    self.marker_lost_time = current_time
                
                self.motor.request_move(self.search_dir, SPD_SEARCH, priority=1)
    
    def process_frame(self, frame, success):
        """Process a frame and update navigation"""
        if not success:
            cv2.putText(frame, "NO VIDEO", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return frame
        
        # Detect markers
        markers = self.detect_markers(frame)
        
        # Draw markers
        if markers:
            for marker in markers:
                corners = marker['corners'].astype(np.int32)
                color = (0, 255, 0)  # Default green
                
                if marker['id'] == 0:
                    color = (0, 0, 255)  # Red for hazard
                elif marker['id'] == self.target_id:
                    color = (0, 255, 255)  # Yellow for target
                
                cv2.polylines(frame, [corners], True, color, 2)
                cv2.putText(frame, str(marker['id']), 
                           (int(corners[0][0]), int(corners[0][1]) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Update state machine in autonomous mode
        if self.mode == Mode.AUTO:
            self.update_state_machine(markers, time.time())
        
        # Draw UI overlay
        self.draw_overlay(frame, markers)
        
        return frame
    
    def draw_overlay(self, frame, markers):
        """Draw navigation overlay"""
        # Status bar
        cv2.rectangle(frame, (0, 0), (320, 70), (0, 0, 0), -1)
        
        # Mode indicator
        mode_color = (0, 255, 0) if self.mode == Mode.AUTO else (0, 165, 255)
        cv2.putText(frame, self.mode.value, (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 1)
        
        # State indicator
        cv2.putText(frame, f"State: {self.state}", (10, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Target info
        if self.target_id:
            target_text = f"Target: {self.target_id}"
            if self.mode == Mode.AUTO:
                target_text += f" | Search: {self.search_dir}"
            cv2.putText(frame, target_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Draw center and deadzone lines
        cv2.line(frame, (X_CENTER, 0), (X_CENTER, 240), (0, 0, 255), 1)
        cv2.line(frame, (X_CENTER - STEER_DEADZONE, 0),
                (X_CENTER - STEER_DEADZONE, 240), (0, 100, 255), 1)
        cv2.line(frame, (X_CENTER + STEER_DEADZONE, 0),
                (X_CENTER + STEER_DEADZONE, 240), (0, 100, 255), 1)
        
        # Show detected markers count
        if markers:
            cv2.putText(frame, f"Markers: {len(markers)}", (220, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

def main():
    print("=" * 60)
    print("WHEELCHAIR AUTONOMOUS NAVIGATION SYSTEM")
    print("=" * 60)
    
    # Initialize components
    vs = VideoStream(VISION_URL)
    brain = SmartBrain()
    
    # --- PATIENCE FIX: Wait for the first valid frame ---
    print("⏳ Waiting for valid camera frames...")
    start_wait = time.time()
    camera_ready = False
    
    while time.time() - start_wait < 10: # Wait up to 10 seconds
        success, frame = vs.read()
        if success and frame is not None and np.sum(frame) > 0:
            camera_ready = True
            break
        time.sleep(0.5)
        print(".")

    if not camera_ready:
        print("\n❌ Camera not available! Check:")
        print("1. Is a browser tab open? (Close it!)")
        print("2. Is the XIAO LED solid yellow?")
        print("3. Is the IP address correct?")
        return
    
    print("\n✅ System Ready & Video Received!")
    
    try:
        while True:
            # Read frame
            success, frame = vs.read()
            
            # Process frame
            display_frame = brain.process_frame(frame, success)
            
            # Display
            cv2.imshow("Autonomous Navigation System", display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            # Mode toggle
            if key == ord('m'):
                brain.mode = Mode.AUTO if brain.mode == Mode.MANUAL else Mode.MANUAL
                brain.motor.request_move("S", 0)
                print(f"Mode: {brain.mode.value}")
            
            # Autonomous mode targets
            if brain.mode == Mode.AUTO:
                if key == ord('1'):
                    brain.target_id = 9
                    print("🎯 Target set: Kitchen (Marker 9)")
                elif key == ord('2'):
                    brain.target_id = 5
                    print("🎯 Target set: Bedroom (Marker 5)")
                elif key == ord('3'):
                    brain.target_id = 3
                    print("🎯 Target set: Hallway (Marker 3)")
                elif key == ord('0'):
                    brain.target_id = 0
                    print("⚠️ Hazard test: Marker 0")
            
            # Manual controls (only in manual mode)
            elif brain.mode == Mode.MANUAL:
                if key == ord('w'):
                    brain.motor.request_move("F", 200)
                elif key == ord('s'):
                    brain.motor.request_move("B", 160)
                elif key == ord('a'):
                    brain.motor.request_move("L", 160)
                elif key == ord('d'):
                    brain.motor.request_move("R", 160)
                elif key == 32:  # Space
                    brain.motor.request_move("S", 0)
            
            # Global controls
            if key == ord('s'):
                brain.motor.request_move("S", 0, priority=10)
                print("🛑 Emergency stop")
            elif key == ord('q'):
                print("Quitting...")
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    finally:
        # Clean shutdown
        brain.motor.request_move("S", 0)
        cv2.destroyAllWindows()
        print("✅ System shutdown complete")

if __name__ == "__main__":
    main()