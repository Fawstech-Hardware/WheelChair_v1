import cv2
import numpy as np
import websocket
import threading
import time
import json
import os
import urllib.request
import queue
import speech_recognition as sr
import pyttsx3
import sys
import signal
import math
from enum import Enum

# --- CONFIGURATION ---
VISION_URL = "http://172.20.11.126/"      
ACTUATOR_WS = "ws://172.20.11.118/ws"

# Tuning
X_CENTER = 160          
STEER_DEADZONE = 35     
STOP_AREA = 12000       # Reduced to stop further from marker
ROTATION_360_TIME = 5.0 # Time for a full circle

# Speed Settings
# Speed Settings
SPD_MAX_AUTO = 140      # ⬇️ Reduced from 170 to reduce blur
SPD_SEARCH = 120        # ⬇️ Reduced from 150
SPD_ALIGN = 100         # ⬇️ Reduced for stable tracking
SPD_KICKSTART = 200

# PID Controller Settings
KP = 0.5   # Proportional Gain
KI = 0.02  # Integral Gain
KD = 0.1   # Derivative Gain
MAX_TURN_DIFF = 100     # Max speed difference between wheels

os.environ['OPENCV_LOG_LEVEL'] = 'OFF'
os.environ['OPENCV_VIDEOIO_DEBUG'] = '0'

# Suppress libjpeg warnings (Corrupt JPEG data messages)
import ctypes
try:
    # Windows: Redirect stderr for libjpeg warnings
    libc = ctypes.CDLL('msvcrt')
except:
    pass

class LocationID(Enum):
    HAZARD = 0
    BATHROOM = 5 
    HALLWAY = 6
    BEDROOM = 3 
    LIVING_ROOM = 7
    KITCHEN = 9

# Navigation Graph: Which locations are connected (for pathfinding)
NAV_GRAPH = {
    LocationID.HALLWAY: [LocationID.BEDROOM, LocationID.LIVING_ROOM, LocationID.BATHROOM],
    LocationID.BEDROOM: [LocationID.HALLWAY],
    LocationID.LIVING_ROOM: [LocationID.HALLWAY, LocationID.KITCHEN],
    LocationID.KITCHEN: [LocationID.LIVING_ROOM],
    LocationID.BATHROOM: [LocationID.HALLWAY],
}

# --- LOGGING SYSTEM (Ported from simple_navigator) ---
class Logger:
    """Unified logging system with different verbosity levels"""
    COLORS = {
        'INFO': '\033[94m',     # Blue
        'SUCCESS': '\033[92m',  # Green
        'WARNING': '\033[93m',  # Yellow
        'ERROR': '\033[91m',    # Red
        'DEBUG': '\033[90m',    # Gray
        'STATE': '\033[95m',    # Purple
        'RESET': '\033[0m'      # Reset
    }
    
    @staticmethod
    def log(level, message, verbose=True):
        """Print log message with timestamp and color"""
        if not verbose: return
        timestamp = time.strftime("%H:%M:%S")
        color = Logger.COLORS.get(level, Logger.COLORS['RESET'])
        print(f"{color}[{timestamp}] [{level:8}] {message}{Logger.COLORS['RESET']}")

# --- 1. NON-BLOCKING VOICE ---
class VoiceInterface:
    def __init__(self, brain):
        self.brain = brain
        self.engine = pyttsx3.init()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.speech_queue = queue.Queue()
        
        threading.Thread(target=self._speech_worker, daemon=True).start()
        threading.Thread(target=self._listen_loop, daemon=True).start()

    def _speech_worker(self):
        """Threaded speech so it doesn't block the motors"""
        while True:
            text = self.speech_queue.get()
            self.engine.say(text)
            self.engine.runAndWait()

    def speak(self, text):
        Logger.log("INFO", f"🗣️ AI: {text}")
        self.speech_queue.put(text)

    def _listen_loop(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            while True:
                try:
                    audio = self.recognizer.listen(source, phrase_time_limit=4)
                    text = self.recognizer.recognize_google(audio).lower()
                    Logger.log("INFO", f"👂 Heard: {text}")
                    if "kitchen" in text: self.brain.set_mission(LocationID.KITCHEN)
                    elif "bedroom" in text: self.brain.set_mission(LocationID.BEDROOM)
                    elif "stop" in text: self.brain.emergency_stop()
                except: pass

# --- 2. THREAD-SAFE MOTOR HAL ---
class MotorController:
    def __init__(self):
        self.ws = None
        self.lock = threading.Lock() # 🔒 Crucial for preventing WinError 10053
        self.last_cmd = "S"
        self.last_left = 0
        self.last_right = 0
        self.last_left = 0
        self.last_right = 0
        self.distance = 0 # 📏 Added ultrasonic feedback
        
        self.connect()
        threading.Thread(target=self.heartbeat, daemon=True).start()
        threading.Thread(target=self._receive_worker, daemon=True).start() # 👂 Start Listener

    def connect(self):
        try:
            self.ws = websocket.create_connection(ACTUATOR_WS, timeout=2)
            Logger.log("SUCCESS", "Actuator Connected")
        except: pass

    def heartbeat(self):
        while True:
            if self.ws:
                try:
                    with self.lock: self.ws.send(json.dumps({"cmd": "H"}))
                except: self.connect()
            time.sleep(0.1)

    def _receive_worker(self):
        """Listen for messages (e.g. distance) from ESP32"""
        while True:
            if self.ws:
                try:
                    # WebSocket recv is blocking, consider timeout/select if robust
                    msg = self.ws.recv() 
                    data = json.loads(msg)
                    if "dist" in data:
                        self.distance = float(data["dist"])
                        # Logger.log("DEBUG", f"📏 Dist: {self.distance}cm")
                except Exception as e:
                    # Logger.log("DEBUG", f"RX Error: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(1)

    def send_differential(self, left_spd, right_spd):
        """
        Sends independent left/right motor speeds.
        Speeds should be between -255 and 255.
        Negative = Backward, Positive = Forward.
        """
        # Optimization: Don't resend if similar
        if (abs(left_spd - self.last_left) < 5 and 
            abs(right_spd - self.last_right) < 5 and
            not (left_spd == 0 and right_spd == 0)):
            return

        left_dir = "F" if left_spd >= 0 else "B"
        right_dir = "F" if right_spd >= 0 else "B"
        
        l_val = min(255, abs(int(left_spd)))
        r_val = min(255, abs(int(right_spd)))

        payload = {
            "cmd": "M",
            "left_dir": left_dir, "left_spd": l_val,
            "right_dir": right_dir, "right_spd": r_val
        }

        try:
            with self.lock:
                self.ws.send(json.dumps(payload))
                self.last_left = left_spd
                self.last_right = right_spd
                # Logger.log("DEBUG", f"Motor: L{l_val} R{r_val}")
        except: pass

    def move(self, direction, speed):
        """ Legacy wrapper for simple moves (Stop, Rotation) """
        if direction == "S":
            self.send_differential(0, 0)
        elif direction == "L": # Rotate Left (Spot Turn)
            self.send_differential(-speed, speed)
        elif direction == "R": # Rotate Right (Spot Turn)
            self.send_differential(speed, -speed)
        elif direction == "F": # Forward
            self.send_differential(speed, speed)
        elif direction == "B": # Backward
            self.send_differential(-speed, -speed)

# --- 3. THREAD-SAFE VIDEO RECEIVER ---
class VideoReceiver:
    def __init__(self, url):
        self.url = url
        self.latest_frame = None
        self.last_frame_time = 0
        self.running = True
        self.lock = threading.Lock()
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        stream = None
        bytes_data = b''
        while self.running:
            try:
                if not stream:
                    stream = urllib.request.urlopen(self.url, timeout=2)
                    Logger.log("SUCCESS", "📷 Vision Node Connected")
                
                chunk = stream.read(4096)
                if not chunk: # Stream ended
                    stream = None
                    continue

                bytes_data += chunk
                
                # Buffer Limit: Prevent lag buildup but allow enough for a frame
                if len(bytes_data) > 65536: 
                    bytes_data = b''
                    continue

                a = bytes_data.find(b'\xff\xd8')
                while a != -1:
                    b = bytes_data.find(b'\xff\xd9', a)
                    if b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        
                        try:
                            # Validate JPEG: must be at least 100 bytes
                            if len(jpg) > 100:
                                # Suppress stderr during decode (libjpeg warnings)
                                import sys, io
                                old_stderr = sys.stderr
                                sys.stderr = io.StringIO()
                                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                                sys.stderr = old_stderr
                                
                                if frame is not None:
                                    with self.lock:
                                        self.latest_frame = frame
                                        self.last_frame_time = time.time()
                        except: pass
                        
                        # Look for next frame in remaining buffer
                        a = bytes_data.find(b'\xff\xd8')
                    else:
                        break # Incomplete frame, wait for more data

            except Exception:
                time.sleep(0.1)
                stream = None

    def get_frame(self):
        with self.lock:
            return self.latest_frame, self.last_frame_time

    def stop(self):
        self.running = False
# --- 4. THE BRAIN ---

class PoseEstimator:
    def __init__(self):
        # Approximate Camera Matrix for 320x240 (Estimated for ~60 deg FOV)
        self.camera_matrix = np.array([
            [320, 0, 160],
            [0, 320, 120],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((4,1)) # Assuming no distortion
        self.marker_length = 0.1 # 10cm markers

    def estimate(self, corners):
        """ Returns rvec, tvec for the first marker """
        c = corners[0] 
        success, rvec, tvec = cv2.solvePnP(
            np.array([
                [-self.marker_length/2,  self.marker_length/2, 0],
                [ self.marker_length/2,  self.marker_length/2, 0],
                [ self.marker_length/2, -self.marker_length/2, 0],
                [-self.marker_length/2, -self.marker_length/2, 0]
            ], dtype=np.float32),
            c,
            self.camera_matrix,
            self.dist_coeffs
        )
        return rvec, tvec, success

    def get_yaw(self, rvec):
        """ Correct Yaw angle (in degrees) from Rotation Vector """
        try:
            rmat, _ = cv2.Rodrigues(rvec)
            # Yaw is rotation around Y-axis (assuming Y-up or Y-down convention of marker)
            # Adjust based on your marker coordinate system. 
            # Usually: arctan2(-R31, sqrt(R32^2 + R33^2)) for pitch, etc.
            # Simple planar yaw: arctan2(R13, R33) or similar.
            # Let's use the one that worked in debug_navigator:
            yaw = np.degrees(np.arctan2(rmat[2, 0], rmat[0, 0])) 
            return yaw
        except: return 0.0

    def draw_axis(self, frame, rvec, tvec):
        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)


# --- 5. STATE MACHINE LOGIC ---
class NavState(Enum):
    IDLE = "IDLE"           # Waiting for command
    SCAN = "SCANNING"       # Rotating to find marker
    ALIGN = "ALIGNING"      # Centering the marker
    APPROACH = "APPROACHING"# Moving forward
    ARRIVED = "ARRIVED"     # Reached destination
    MANUAL = "MANUAL"       # User control

class SmartNavigator:
    def __init__(self):
        self.motor = MotorController()
        self.voice = VoiceInterface(self)
        self.video = VideoReceiver(VISION_URL)
        self.pose = PoseEstimator() # 3D Pose Logic
        
        # State & Logic
        self.state = NavState.IDLE
        self.current_loc = LocationID.HALLWAY
        self.target_loc = None
        self.manual_mode = False
        
        
        # Tracking
        self.last_marker_time = 0
        self.scan_start_time = 0
        self.last_log_time = 0
        
        # Statistics & FPS
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        self.total_markers_detected = 0
        self.state_changes = 0
        self.previous_state = NavState.IDLE
        
        # 3D Visuals
        self.current_rvec = None
        self.current_tvec = None

        # Path Planning
        self.waypoint_queue = []  # List of LocationIDs to visit
        self.final_destination = None  # The ultimate goal

        # PID
        self.error_integral = 0
        self.last_error = 0

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Tuning for Stability
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10

    def log(self, state, msg):
        """Structured logging for terminal clarity"""
        if time.time() - self.last_log_time > 0.5 or "Found" in msg or "Arrived" in msg or "Lost" in msg: 
            Logger.log("STATE", f"[{state.value:<10}] {msg}")
            self.last_log_time = time.time()

    def find_path(self, start, goal):
        """Find shortest path between locations using BFS"""
        Logger.log("DEBUG", f"Finding path: {start.name} → {goal.name}")
        from collections import deque
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            current, path = queue.popleft()
            if current == goal:
                result = path[1:]  # Exclude start location
                Logger.log("DEBUG", f"Path found: {[loc.name for loc in result]}")
                return result
            
            for neighbor in NAV_GRAPH.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        Logger.log("WARNING", "No path found!")
        return []  # No path found

    def set_mission(self, target_loc):
        """Start navigation with automatic path planning"""
        # Calculate path from current location to target
        path = self.find_path(self.current_loc, target_loc)
        
        if not path:
            # Direct navigation (no path needed or already at location)
            if self.current_loc == target_loc:
                self.voice.speak(f"Already at {target_loc.name}")
                return
            path = [target_loc]  # Try direct if no graph path
        
        # Store the path
        self.waypoint_queue = path.copy()
        self.final_destination = target_loc
        
        # Start with first waypoint
        self.target_loc = self.waypoint_queue.pop(0)
        
        # Announce the route
        route_names = " → ".join([loc.name for loc in [self.current_loc] + [self.target_loc] + self.waypoint_queue])
        self.voice.speak(f"Route: {route_names}")
        
        self.change_state(NavState.SCAN)
        self.scan_start_time = time.time()
        Logger.log("SUCCESS", f"🚀 PATH: {route_names}")
        
    def change_state(self, new_state):
        if self.state != new_state:
            self.previous_state = self.state
            self.state = new_state
            self.state_changes += 1
            Logger.log("STATE", f"{self.previous_state.value} -> {new_state.value}")

    def calculate_pid(self, error):
        self.error_integral += error
        self.error_integral = np.clip(self.error_integral, -10, 10)
        derivative = error - self.last_error
        self.last_error = error
        return (KP * error) + (KI * self.error_integral) + (KD * derivative)

    def emergency_stop(self):
        self.change_state(NavState.IDLE)
        self.target_loc = None
        self.motor.move("S", 0)
        self.voice.speak("Stopped.")
        Logger.log("WARNING", "⚠️ EMERGENCY STOP TRIGGERED")

    # --- STATE HANDLERS ---
    
    def handle_scan(self, id_list, corners):
        """State: SCAN - Rotate until target seen"""
        # If target seen -> Switch to ALIGN
        if self.target_loc.value in id_list:
            self.log(NavState.SCAN, f"👀 Found Target {self.target_loc.name}! Aligning...")
            self.last_marker_time = time.time() # Reset time on find
            self.change_state(NavState.ALIGN)
            self.motor.move("S", 0)
            return

        # If Search Timeout (10s) -> Random adjust or reverse direction (Simple logic: just Keep Rotating Right)
        elapsed = time.time() - self.scan_start_time
        if elapsed < 10:
            self.motor.move("R", SPD_SEARCH)
            self.log(NavState.SCAN, f"🔄 Rotating Right... ({int(elapsed)}s)")
        else:
             self.motor.move("L", SPD_SEARCH) # Switch direction
             self.log(NavState.SCAN, "🔄 Switching to Left Scan...")

    def handle_align(self, id_list, corners):
        """State: ALIGN - Face the marker using 3D Bearing (tvec)"""
        if self.target_loc.value not in id_list:
            if time.time() - self.last_marker_time > 5.0: # 5 sec timeout
                self.log(NavState.ALIGN, "❌ Target Lost! Back to SCAN.")
                self.change_state(NavState.SCAN)
                self.scan_start_time = time.time()
            # Clear 3D visuals if lost temp
            self.current_rvec = None 
            self.current_tvec = None
            return

        self.last_marker_time = time.time()
        idx = id_list.index(self.target_loc.value)
        
        # 3D POSE ESTIMATION
        rvec, tvec, success = self.pose.estimate([corners[idx]])
        if success:
            self.current_rvec = rvec
            self.current_tvec = tvec
            
            # 1. Calculate BEARING (Angle TO the target)
            # tvec is [x, y, z] in camera frame. 
            # Bearing = atan2(x, z). 
            # If x is positive (right), bearing is positive. We need to turn RIGHT.
            # (Note: Camera coords: X right, Y down, Z forward)
            tx = tvec[0][0]
            tz = tvec[2][0]
            bearing = math.degrees(math.atan2(tx, tz))
            
            # 2. Use Bearing for alignment error
            # If bearing is +10 deg (Right), we turn Right.
            # Error = bearing / 45.0
            error = bearing / 20.0 # Normalize 20 deg -> 1.0 (Sensitive)
            
            # Also get Surface Yaw for logging
            surface_yaw = self.pose.get_yaw(rvec)

            if abs(bearing) < 5.0: # ⬆️ Relaxed Deadzone (was 2.0)
                self.log(NavState.ALIGN, "✅ Aligned (Bearing)! Approaching...")
                self.change_state(NavState.APPROACH)
                self.motor.move("S", 0)
            else:
                pid_out = self.calculate_pid(error)
                # Turn Logic: If bearing + (Target Right), Turn Right ("R")
                # Wait.. my move("R") means "Rotate Right" (Clockwise).
                # If Target is Right, we rotate Right. Correct.
                
                if pid_out > 0: self.motor.move("R", SPD_ALIGN) 
                else: self.motor.move("L", SPD_ALIGN)
                
                self.log(NavState.ALIGN, f"🎯 Brg: {bearing:.1f}° (Yaw: {surface_yaw:.0f}°) Err: {error:.2f}")
        else:
            # Fallback - should ideally stick to 2D
             self.log(NavState.ALIGN, "⚠️ Pose Failed - Retry")


    def handle_approach(self, id_list, corners):
        """State: APPROACH - Drive forward ONLY if Aligned (3D)"""
        if self.target_loc.value not in id_list:
            if time.time() - self.last_marker_time > 1.0:
                self.log(NavState.APPROACH, "❌ Target Lost!")
                self.change_state(NavState.ALIGN)
            self.current_rvec = None # Clear 3D visuals if target lost
            self.current_tvec = None
            return
        
        self.last_marker_time = time.time()
        idx = id_list.index(self.target_loc.value)
        
        # 3D POSE CHECK
        rvec, tvec, success = self.pose.estimate([corners[idx]])
        if success:
            self.current_rvec = rvec
            self.current_tvec = tvec
            # 1. Calculate BEARING (Angle TO the target)
            tx = tvec[0][0]
            tz = tvec[2][0]
            bearing = math.degrees(math.atan2(tx, tz))
            
            # 2. Steer based on BEARING (Keep target in center)
            # If bearing is positive (Target is Right), we turn Right.
            error = bearing / 45.0
            
            if abs(bearing) > 15: # If target drift > 15 deg, Re-align (was 20)
                self.log(NavState.APPROACH, f"⚠️ Drift ({bearing:.1f}°) -> Re-aligning")
                self.change_state(NavState.ALIGN)
                self.motor.move("S", 0)
                return

            # ARRIVED CHECK
            area = cv2.contourArea(corners[idx])
            dist = self.motor.distance
            
            if (area > STOP_AREA) or (0 < dist < 40):  # Stop 40cm away
                 self.change_state(NavState.ARRIVED)
                 self.motor.move("S", 0)
                 self.log(NavState.APPROACH, "🏁 Arrived Safely!")
                 return

            # Drive with angular correction (Steer to 0 Bearing)
            pid_out = self.calculate_pid(error)
            turn_adj = pid_out * MAX_TURN_DIFF
            
            # Bearing > 0 (Right) -> turn_adj > 0
            # To turn right: Increase Left, Decrease Right
            l_spd = SPD_MAX_AUTO + turn_adj 
            r_spd = SPD_MAX_AUTO - turn_adj
            
            self.motor.send_differential(l_spd, r_spd)
            self.log(NavState.APPROACH, f"🚀 Brg:{bearing:.1f}° Dist:{dist}cm")


    def run(self):
        Logger.log("SUCCESS", "🤖 AI Navigator v7.0 (Stats + Enhanced HUD) Ready")
        
        while True:
            frame, last_time = self.video.get_frame()
            
            # Watchdog
            if time.time() - last_time > 3.0:
                 if self.state != NavState.IDLE: self.emergency_stop()
                 time.sleep(0.1); continue

            if frame is None: continue

            try:
                # 1. Stats & FPS
                self.frame_count += 1
                curr_t = time.time()
                if curr_t - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count
                    self.frame_count = 0
                    self.last_fps_time = curr_t

                # 2. Detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.detector.detectMarkers(gray)
                id_list = list(ids.flatten()) if ids is not None else []
                self.total_markers_detected += len(id_list)
                
                if ids is not None: 
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Reset 3D visuals for current frame before handlers update them
                self.current_rvec = None 
                self.current_tvec = None

                # 3. Safety
                if LocationID.HAZARD.value in id_list:
                    self.emergency_stop()
                    cv2.putText(frame, "HAZARD!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 3)

                # 4. State Machine Execution
                if self.manual_mode:
                    # Manual Logic handled by Key Input
                    pass
                elif self.state == NavState.SCAN:
                    self.handle_scan(id_list, corners)
                elif self.state == NavState.ALIGN:
                    self.handle_align(id_list, corners)
                elif self.state == NavState.APPROACH:
                    self.handle_approach(id_list, corners)
                elif self.state == NavState.ARRIVED:
                    # Update current location
                    self.current_loc = self.target_loc
                    
                    # Check if there are more waypoints to visit
                    if self.waypoint_queue:
                        # Continue to next waypoint
                        self.voice.speak(f"Reached {self.target_loc.name}. Continuing...")
                        self.target_loc = self.waypoint_queue.pop(0)
                        Logger.log("INFO", f"📍 Next waypoint: {self.target_loc.name}")
                        self.change_state(NavState.SCAN)
                        self.scan_start_time = time.time()
                    else:
                        # Final destination reached
                        self.voice.speak(f"Arrived at {self.target_loc.name}")
                        Logger.log("SUCCESS", f"🏁 MISSION COMPLETE: {self.target_loc.name}")
                        self.target_loc = None
                        self.final_destination = None
                        self.change_state(NavState.IDLE)
                
                # 5. Enhanced HUD (Smaller overlay)
                overlay = frame.copy()
                cv2.rectangle(overlay, (0,0), (320, 50), (0,0,0), -1)
                frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
                
                # Line 1: State & FPS
                state_col = (0,255,0) if self.state != NavState.IDLE else (200,200,200)
                if self.state == NavState.SCAN: state_col = (0, 255, 255)
                
                cv2.putText(frame, f"ST: {self.state.value}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, state_col, 1)
                cv2.putText(frame, f"FPS: {self.fps}", (240, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                # Line 2: Target
                t_name = self.target_loc.name if self.target_loc else "NONE"
                cv2.putText(frame, f"TGT: {t_name}", (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)

                # Line 3: Motor & Speed
                l_spd = int(self.motor.last_left)
                r_spd = int(self.motor.last_right)
                cv2.putText(frame, f"L:{l_spd} R:{r_spd}", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,100,255), 1)

                # Draw 3D Axis (Post-calculation)
                if self.current_rvec is not None:
                    try:
                        self.pose.draw_axis(frame, self.current_rvec, self.current_tvec)
                        yaw = self.pose.get_yaw(self.current_rvec)
                        cv2.putText(frame, f"YAW: {yaw:.1f}", (240, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
                    except: pass

                # Center Line & Thresholds
                cx = X_CENTER
                cv2.line(frame, (cx, 0), (cx, 240), (0,0,255), 1)
                cv2.line(frame, (cx-STEER_DEADZONE, 0), (cx-STEER_DEADZONE, 240), (50,50,50), 1)
                cv2.line(frame, (cx+STEER_DEADZONE, 0), (cx+STEER_DEADZONE, 240), (50,50,50), 1)
                
                cv2.imshow("AI Vision", frame)

                # 6. Input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): 
                    self.motor.move("S", 0); break
                elif key == ord('m'):
                    self.manual_mode = not self.manual_mode
                    self.change_state(NavState.MANUAL if self.manual_mode else NavState.IDLE)
                    self.motor.move("S", 0)
                    Logger.log("INFO", f"🔄 Manual Mode: {self.manual_mode}")
                
                if self.manual_mode:
                    if key == ord('w'): self.motor.move("F", SPD_MAX_AUTO)
                    elif key == ord('s'): self.motor.move("B", SPD_MAX_AUTO)
                    elif key == ord('a'): self.motor.move("L", SPD_MAX_AUTO)
                    elif key == ord('d'): self.motor.move("R", SPD_MAX_AUTO)
                    elif key == 32: self.motor.move("S", 0)
                
                # Auto Triggers
                elif not self.manual_mode:
                    if key == ord('k'): self.set_mission(LocationID.KITCHEN)
                    elif key == ord('b'): self.set_mission(LocationID.BEDROOM)
                    elif key == ord('h'): self.set_mission(LocationID.HALLWAY)
                    elif key == ord('l'): self.set_mission(LocationID.LIVING_ROOM)
                    elif key == ord('t'): self.set_mission(LocationID.BATHROOM)

            except Exception as e:
                Logger.log("ERROR", f"Loop Error: {e}")
                time.sleep(0.1)
        
        self.motor.move("S", 0)
        self.video.stop()
        cv2.destroyAllWindows()
        self.print_statistics()
    
    def print_statistics(self):
        print("\n" + "="*60)
        print("📊 SESSION STATISTICS")
        print("="*60)
        print(f"Total state changes: {self.state_changes}")
        print(f"Total markers detected: {self.total_markers_detected}")
        print(f"Final state: {self.state.value}")
        print(f"Average FPS: {self.fps}")
        print("="*60)

if __name__ == "__main__":
    SmartNavigator().run()
