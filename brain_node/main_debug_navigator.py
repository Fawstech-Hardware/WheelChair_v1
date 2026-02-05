import cv2
import numpy as np
import websocket
import threading
import time
import json
import os
import urllib.request
import queue
import math
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List

# ============================================================================
# CONFIGURATION
# ============================================================================
VISION_URL = "http://192.168.0.126/"
ACTUATOR_WS = "ws://192.168.0.118/ws"

# Camera settings
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
X_CENTER = IMAGE_WIDTH // 2

# Navigation tuning
# Navigation tuning
SCAN_SPEED = 140      # ⬆️ Increased from 100
ALIGN_SPEED = 150     # ⬆️ Increased from 120 (friction)
APPROACH_SPEED = 160  # ⬆️ Increased from 140
STOP_AREA = 18000
ALIGN_THRESHOLD_DEG = 15  # ⬆️ Relaxed from 10 to 15 to allow Approach sooner
DISTANCE_STOP_CM = 25

# Timeouts
SCAN_TIMEOUT = 8.0
LOST_TIMEOUT = 2.0

# Logging verbosity
LOG_LEVEL = "DETAILED"  # MINIMAL, DETAILED, DEBUG

# ============================================================================
# DATA STRUCTURES
# ============================================================================
@dataclass
class MarkerData:
    id: int
    corners: np.ndarray
    center_x: float
    center_y: float
    area: float

@dataclass  
class MotorCommand:
    left: int  # -255 to 255
    right: int  # -255 to 255

class LocationID(Enum):
    HAZARD = 0
    BEDROOM = 3
    BATHROOM = 5
    HALLWAY = 6
    LIVING_ROOM = 7
    KITCHEN = 9

class NavState(Enum):
    IDLE = "IDLE"
    SCAN_RIGHT = "SCAN_RIGHT"
    SCAN_LEFT = "SCAN_LEFT"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    ARRIVED = "ARRIVED"
    MANUAL = "MANUAL"
    ERROR = "ERROR"

# ============================================================================
# ENHANCED LOGGING SYSTEM
# ============================================================================
class DebugLogger:
    """Comprehensive logging with timestamps and colors"""
    
    COLORS = {
        'TIMESTAMP': '\033[90m',
        'STATE': '\033[95m',
        'MARKER': '\033[96m',
        'MOTOR': '\033[93m',
        'VISION': '\033[94m',
        'SUCCESS': '\033[92m',
        'WARNING': '\033[93m',
        'ERROR': '\033[91m',
        'DEBUG': '\033[90m',
        'RESET': '\033[0m'
    }
    
    # State machine visualizer
    STATE_SYMBOLS = {
        NavState.IDLE: "⏸️",
        NavState.SCAN_RIGHT: "🔍↪️",
        NavState.SCAN_LEFT: "🔍↩️",
        NavState.ALIGN: "🎯",
        NavState.APPROACH: "🚶",
        NavState.ARRIVED: "✅",
        NavState.MANUAL: "👤",
        NavState.ERROR: "❌"
    }
    
    @staticmethod
    def log_state_change(old_state: NavState, new_state: NavState, reason: str = ""):
        """Log state transitions prominently"""
        timestamp = time.strftime("%H:%M:%S")
        old_symbol = DebugLogger.STATE_SYMBOLS.get(old_state, "")
        new_symbol = DebugLogger.STATE_SYMBOLS.get(new_state, "")
        
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{timestamp}] "
              f"{DebugLogger.COLORS['STATE']}{old_symbol}{old_state.value:12} → "
              f"{new_symbol}{new_state.value:12} {DebugLogger.COLORS['RESET']}"
              f"{reason}")
    
    @staticmethod
    def log_marker_detection(markers: List[MarkerData], target_id: Optional[int] = None):
        """Log marker detection details"""
        if not markers:
            return
        
        ids = [m.id for m in markers]
        areas = [int(m.area) for m in markers]
        
        # Show all detected markers
        marker_str = ", ".join([f"ID:{id}({area})" for id, area in zip(ids, areas)])
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
              f"{DebugLogger.COLORS['MARKER']}👁️ Markers: [{marker_str}]")
        
        # Highlight target if present
        if target_id in ids:
            idx = ids.index(target_id)
            print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
                  f"{DebugLogger.COLORS['MARKER']}  🎯 Target ID:{target_id} "
                  f"at ({markers[idx].center_x:.1f}, {markers[idx].center_y:.1f}) "
                  f"area:{areas[idx]}")
    
    @staticmethod
    def log_motor_command(cmd: MotorCommand):
        """Log motor commands with direction arrows"""
        left_dir = "←" if cmd.left < 0 else "→" if cmd.left > 0 else "•"
        right_dir = "←" if cmd.right < 0 else "→" if cmd.right > 0 else "•"
        
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
              f"{DebugLogger.COLORS['MOTOR']}⚙️ Motors: {left_dir}L:{abs(cmd.left):3d} "
              f"{right_dir}R:{abs(cmd.right):3d}")
    
    @staticmethod
    def log_vision_stats(fps: int, frame_lag: float):
        """Log vision system statistics"""
        if LOG_LEVEL == "DEBUG":
            print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
                  f"{DebugLogger.COLORS['VISION']}📊 FPS:{fps:2d} Lag:{frame_lag:.3f}s")
    
    @staticmethod
    def log_pose_estimation(yaw: float, distance: float):
        """Log 3D pose estimation results"""
        if LOG_LEVEL == "DEBUG":
            print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
                  f"{DebugLogger.COLORS['VISION']}📐 Yaw:{yaw:6.1f}° Dist:{distance:5.1f}cm")
    
    @staticmethod
    def log_info(message: str):
        """General info logging"""
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
              f"{DebugLogger.COLORS['DEBUG']}💡 {message}")
    
    @staticmethod
    def log_warning(message: str):
        """Warning messages"""
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
              f"{DebugLogger.COLORS['WARNING']}⚠️ {message}")
    
    @staticmethod
    def log_error(message: str):
        """Error messages"""
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
              f"{DebugLogger.COLORS['ERROR']}❌ {message}")
    
    @staticmethod
    def log_success(message: str):
        """Success messages"""
        print(f"{DebugLogger.COLORS['TIMESTAMP']}[{time.strftime('%H:%M:%S')}] "
              f"{DebugLogger.COLORS['SUCCESS']}✅ {message}")

# ============================================================================
# 3D POSE ESTIMATOR WITH DEBUGGING
# ============================================================================
class PoseEstimatorDebug:
    """3D pose estimator with debugging visualization"""
    
    def __init__(self):
        # Camera calibration (estimated for 320x240, ~60° FOV)
        self.camera_matrix = np.array([
            [300, 0, 160],    # fx, 0, cx
            [0, 300, 120],    # 0, fy, cy
            [0, 0, 1]         # 0, 0, 1
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4, 1))
        self.marker_size = 0.1  # 10cm markers
        
        # 3D coordinate system for marker corners
        self.marker_points = np.array([
            [-self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)
    
    def estimate_pose(self, corners: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """Estimate 3D pose of marker"""
        try:
            success, rvec, tvec = cv2.solvePnP(
                self.marker_points,
                corners.astype(np.float32),
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                # Convert rotation vector to rotation matrix
                rmat, _ = cv2.Rodrigues(rvec)
                
                # Calculate yaw angle (rotation around Y axis)
                yaw = np.degrees(np.arctan2(rmat[2, 0], rmat[0, 0]))
                
                # Calculate distance (Z coordinate)
                distance = np.linalg.norm(tvec)
                
                return rvec, tvec, yaw, distance, True
                
        except Exception as e:
            DebugLogger.log_warning(f"Pose estimation failed: {str(e)}")
        
        return None, None, 0, 0, False
    
    def draw_3d_axes(self, frame: np.ndarray, rvec: np.ndarray, tvec: np.ndarray):
        """Draw 3D coordinate axes on marker"""
        try:
            # Draw axes (5cm length)
            axis_length = 0.05
            axes_points = np.array([
                [0, 0, 0],
                [axis_length, 0, 0],    # X axis (red)
                [0, axis_length, 0],    # Y axis (green)
                [0, 0, -axis_length]    # Z axis (blue, negative for camera frame)
            ], dtype=np.float32)
            
            # Project 3D points to 2D
            img_points, _ = cv2.projectPoints(
                axes_points, rvec, tvec,
                self.camera_matrix, self.dist_coeffs
            )
            
            img_points = img_points.reshape(-1, 2).astype(int)
            
            # Draw axes
            cv2.line(frame, tuple(img_points[0]), tuple(img_points[1]), (0, 0, 255), 2)  # X: Red
            cv2.line(frame, tuple(img_points[0]), tuple(img_points[2]), (0, 255, 0), 2)  # Y: Green
            cv2.line(frame, tuple(img_points[0]), tuple(img_points[3]), (255, 0, 0), 2)  # Z: Blue
            
        except Exception as e:
            pass  # Silently fail on axis drawing errors
    
    def calculate_2d_alignment_error(self, center_x: float) -> float:
        """Calculate 2D alignment error (normalized -1 to 1)"""
        return (center_x - X_CENTER) / X_CENTER

# ============================================================================
# SIMPLE MOTOR CONTROLLER
# ============================================================================
class SimpleMotorController:
    """Simple motor controller with command queuing"""
    
    def __init__(self):
        self.ws = None
        self.last_command = MotorCommand(0, 0)
        self.distance_cm = 999
        self.lock = threading.Lock()
        
        self.connect()
        threading.Thread(target=self._heartbeat, daemon=True).start()
        threading.Thread(target=self._receive_loop, daemon=True).start()
    
    def connect(self):
        """Connect to WebSocket"""
        try:
            self.ws = websocket.create_connection(ACTUATOR_WS, timeout=2)
            DebugLogger.log_success("Actuator connected")
        except Exception as e:
            DebugLogger.log_error(f"Actuator connection failed: {e}")
    
    def _heartbeat(self):
        """Send heartbeat to keep connection alive"""
        while True:
            if self.ws:
                try:
                    with self.lock:
                        self.ws.send(json.dumps({"cmd": "H"}))
                except:
                    self.connect()
            time.sleep(0.1)
    
    def _receive_loop(self):
        """Receive data from actuator (e.g., ultrasonic distance)"""
        while True:
            if self.ws:
                try:
                    msg = self.ws.recv()
                    data = json.loads(msg)
                    if "dist" in data:
                        self.distance_cm = float(data["dist"])
                except:
                    pass
            time.sleep(0.05)
    
    def send_command(self, cmd: MotorCommand):
        """Send motor command if changed significantly"""
        # Only send if changed by more than 5 units
        if (abs(cmd.left - self.last_command.left) < 5 and 
            abs(cmd.right - self.last_command.right) < 5 and
            not (cmd.left == 0 and cmd.right == 0)):
            return
        
        self.last_command = cmd
        
        # Build command
        left_dir = "F" if cmd.left >= 0 else "B"
        right_dir = "F" if cmd.right >= 0 else "B"
        
        payload = {
            "cmd": "M",
            "left_dir": left_dir,
            "left_spd": abs(cmd.left),
            "right_dir": right_dir,
            "right_spd": abs(cmd.right)
        }
        
        # Send command
        if self.ws:
            try:
                with self.lock:
                    self.ws.send(json.dumps(payload))
                DebugLogger.log_motor_command(cmd)
            except Exception as e:
                DebugLogger.log_error(f"Failed to send motor command: {e}")
    
    def stop(self):
        """Stop all motors"""
        self.send_command(MotorCommand(0, 0))

# ============================================================================
# VIDEO STREAM WITH DIAGNOSTICS
# ============================================================================
class DiagnosticVideoStream:
    """Video stream with frame timing diagnostics"""
    
    def __init__(self, url: str):
        self.url = url
        self.frame = None
        self.frame_time = 0
        self.running = True
        self.frame_count = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.lock = threading.Lock()
        
        threading.Thread(target=self._update_loop, daemon=True).start()
        
        # Wait for connection
        timeout = time.time() + 5
        while self.frame is None and time.time() < timeout:
            time.sleep(0.1)
        
        if self.frame is not None:
            DebugLogger.log_success(f"Video stream: {self.frame.shape[1]}x{self.frame.shape[0]}")
        else:
            DebugLogger.log_error("Video stream failed to initialize")
    
    def _update_loop(self):
        """Main update loop"""
        stream = None
        bytes_data = b''
        
        while self.running:
            try:
                if stream is None:
                    stream = urllib.request.urlopen(self.url, timeout=2)
                
                # Read larger chunks for stability
                chunk = stream.read(4096)
                if len(chunk) == 0:
                   # Reconnect if stream ends
                   stream = None
                   continue
                   
                bytes_data += chunk
                
                # Look for JPEG Start/End markers
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
                
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    
                    try:
                        # Decode frame
                        nparr = np.frombuffer(jpg, np.uint8)
                        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        
                        if frame is not None:
                            with self.lock:
                                self.frame = frame
                                self.frame_time = time.time()
                                self.frame_count += 1
                            
                            # Calculate FPS
                            current_time = time.time()
                            if current_time - self.last_fps_time >= 1.0:
                                self.fps = self.frame_count
                                self.frame_count = 0
                                self.last_fps_time = current_time
                    except: pass # Ignore partial frames
                    
                # Buffer maintenance to prevent memory leaks if stream desyncs
                if len(bytes_data) > 65536:
                   bytes_data = b'' # Flush buffer
                    
            except Exception as e:
                # DebugLogger.log_warning(f"Stream error: {str(e)}")
                stream = None
                time.sleep(0.5)
    
    def get_frame(self) -> Tuple[Optional[np.ndarray], float]:
        """Get latest frame with timestamp"""
        with self.lock:
            if self.frame is not None:
                frame_lag = time.time() - self.frame_time
                return self.frame.copy(), frame_lag
            return None, 999

# ============================================================================
# MARKER DETECTOR WITH VISUAL DEBUGGING
# ============================================================================
class MarkerDetectorDebug:
    """Marker detector with visual debugging overlays"""
    
    def __init__(self):
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        # Tune parameters for better detection
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
    
    def detect(self, frame: np.ndarray) -> List[MarkerData]:
        """Detect markers and return structured data"""
        if frame is None:
            return []
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Optional: Apply histogram equalization for better contrast
        gray = cv2.equalizeHist(gray)
        
        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        markers = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_corners = corners[i][0]
                
                # Calculate center
                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])
                
                # Calculate area
                area = cv2.contourArea(marker_corners)
                
                markers.append(MarkerData(
                    id=int(marker_id),
                    corners=marker_corners,
                    center_x=float(center_x),
                    center_y=float(center_y),
                    area=float(area)
                ))
        
        return markers
    
    def draw_detection_overlay(self, frame: np.ndarray, markers: List[MarkerData], 
                               target_id: Optional[int] = None):
        """Draw visual debugging overlays"""
        
        for marker in markers:
            # Draw bounding box
            corners_int = marker.corners.astype(np.int32)
            
            # Color based on whether it's the target
            if marker.id == target_id:
                color = (255, 0, 0)  # Blue for target
                thickness = 2
            else:
                color = (0, 255, 0)  # Green for other markers
                thickness = 1
            
            cv2.polylines(frame, [corners_int], True, color, thickness)
            
            # Draw center point
            center = (int(marker.center_x), int(marker.center_y))
            cv2.circle(frame, center, 4, color, -1)
            
            # Draw ID and area
            text = f"ID:{marker.id} A:{int(marker.area)}"
            cv2.putText(frame, text, (center[0] + 10, center[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Draw center crosshair for target
            if marker.id == target_id:
                cv2.line(frame, (center[0]-10, center[1]), 
                        (center[0]+10, center[1]), (0, 255, 255), 1)
                cv2.line(frame, (center[0], center[1]-10), 
                        (center[0], center[1]+10), (0, 255, 255), 1)

# ============================================================================
# MAIN NAVIGATION CONTROLLER WITH DEBUGGING
# ============================================================================
class DebugNavigator:
    """Navigation controller with comprehensive debugging"""
    
    def __init__(self):
        # Initialize components
        self.video = DiagnosticVideoStream(VISION_URL)
        self.motors = SimpleMotorController()
        self.detector = MarkerDetectorDebug()
        self.pose_estimator = PoseEstimatorDebug()
        
        # State
        self.state = NavState.IDLE
        self.previous_state = NavState.IDLE
        
        # Navigation
        self.target_id = None
        self.target_name = ""
        self.current_location = LocationID.HALLWAY
        
        # Timing
        self.state_start_time = time.time()
        self.last_marker_time = 0
        self.scan_start_time = 0
        self.scan_direction = "right"
        
        # Manual mode
        self.manual_mode = False
        
        # Statistics
        self.total_markers = 0
        self.state_changes = 0
        
        DebugLogger.log_success("Debug Navigator initialized")
        self._print_controls()
    
    def _print_controls(self):
        """Print control instructions"""
        print("\n" + "="*70)
        print("🎮 DEBUG NAVIGATION CONTROLS")
        print("="*70)
        print("K: Go to Kitchen (ID 9)")
        print("B: Go to Bedroom (ID 3)")
        print("H: Go to Hallway (ID 6)")
        print("S: Emergency Stop")
        print("M: Toggle Manual/Auto mode")
        print("Q: Quit")
        print("\n📊 DEBUG DISPLAY:")
        print("Green boxes: Detected markers")
        print("Blue boxes: Target marker")
        print("Yellow crosshair: Target center")
        print("3D axes: Red(X), Green(Y), Blue(Z)")
        print("Red line: Screen center")
        print("="*70 + "\n")
    
    def set_target(self, location_id: LocationID):
        """Set navigation target"""
        self.target_id = location_id.value
        self.target_name = location_id.name
        
        DebugLogger.log_success(f"New target: {self.target_name} (ID:{self.target_id})")
        
        # Start scanning to the right
        self.state = NavState.SCAN_RIGHT
        self.scan_start_time = time.time()
        self.scan_direction = "right"
        
        DebugLogger.log_info(f"Starting scan to find marker {self.target_id}")
    
    def _change_state(self, new_state: NavState, reason: str = ""):
        """Change state with logging"""
        if new_state != self.state:
            self.previous_state = self.state
            self.state = new_state
            self.state_start_time = time.time()
            self.state_changes += 1
            
            DebugLogger.log_state_change(self.previous_state, self.state, reason)
    
    def _handle_scan(self, markers: List[MarkerData]):
        """Handle SCAN state"""
        current_time = time.time()
        
        # Check if we found the target
        target_marker = next((m for m in markers if m.id == self.target_id), None)
        
        if target_marker:
            DebugLogger.log_info(f"Target found! Starting alignment")
            self._change_state(NavState.ALIGN, "Target detected")
            self.last_marker_time = current_time
            return
        
        # Check scan timeout (switch direction)
        if current_time - self.scan_start_time > SCAN_TIMEOUT:
            if self.state == NavState.SCAN_RIGHT:
                self._change_state(NavState.SCAN_LEFT, "Scan timeout, switching direction")
                self.scan_start_time = current_time
            else:
                self._change_state(NavState.SCAN_RIGHT, "Scan timeout, switching direction")
                self.scan_start_time = current_time
        
        # Apply rotation based on scan direction
        if self.state == NavState.SCAN_RIGHT:
            self.motors.send_command(MotorCommand(SCAN_SPEED, -SCAN_SPEED))  # Rotate right
            DebugLogger.log_info(f"Scanning right...")
        else:  # SCAN_LEFT
            self.motors.send_command(MotorCommand(-SCAN_SPEED, SCAN_SPEED))  # Rotate left
            DebugLogger.log_info(f"Scanning left...")
    
    def _handle_align(self, markers: List[MarkerData]):
        """Handle ALIGN state"""
        current_time = time.time()
        
        # Find target marker
        target_marker = next((m for m in markers if m.id == self.target_id), None)
        
        if not target_marker:
            # Lost marker
            if current_time - self.last_marker_time > LOST_TIMEOUT:
                DebugLogger.log_warning("Lost target during alignment")
                self._change_state(NavState.SCAN_RIGHT, "Target lost")
                self.scan_start_time = current_time
            return
        
        self.last_marker_time = current_time
        
        # Estimate 3D pose
        rvec, tvec, yaw, distance, success = self.pose_estimator.estimate_pose(target_marker.corners)
        
        if success:
            # Calculate 2D alignment error
            error_2d = self.pose_estimator.calculate_2d_alignment_error(target_marker.center_x)
            
            # Debug logging
            if LOG_LEVEL == "DEBUG":
                DebugLogger.log_pose_estimation(yaw, distance * 100)  # Convert to cm
            
            # Check if aligned (using both 2D and 3D criteria)
            aligned_2d = abs(error_2d) < 0.1  # Within 10% of center
            aligned_3d = abs(yaw) < ALIGN_THRESHOLD_DEG
            
            if aligned_2d and aligned_3d:
                DebugLogger.log_info(f"Aligned! 2D error:{error_2d:.2f}, Yaw:{yaw:.1f}°")
                self._change_state(NavState.APPROACH, "Successfully aligned")
                self.motors.stop()
            else:
                # Apply correction based on yaw angle
                if yaw > ALIGN_THRESHOLD_DEG / 2:
                    # Marker is to the right, turn left
                    self.motors.send_command(MotorCommand(-ALIGN_SPEED, ALIGN_SPEED))
                    DebugLogger.log_info(f"Aligning left (yaw:{yaw:.1f}°)")
                elif yaw < -ALIGN_THRESHOLD_DEG / 2:
                    # Marker is to the left, turn right
                    self.motors.send_command(MotorCommand(ALIGN_SPEED, -ALIGN_SPEED))
                    DebugLogger.log_info(f"Aligning right (yaw:{yaw:.1f}°)")
                else:
                    # Close enough, use 2D alignment
                    if error_2d > 0.05:
                        self.motors.send_command(MotorCommand(ALIGN_SPEED, -ALIGN_SPEED))
                    elif error_2d < -0.05:
                        self.motors.send_command(MotorCommand(-ALIGN_SPEED, ALIGN_SPEED))
                    else:
                        self.motors.stop()
        
        else:
            # Fall back to 2D alignment if 3D fails
            error_2d = self.pose_estimator.calculate_2d_alignment_error(target_marker.center_x)
            
            if abs(error_2d) < 0.05:  # Within 5% of center
                DebugLogger.log_info("Aligned (2D fallback)")
                self._change_state(NavState.APPROACH, "2D alignment successful")
                self.motors.stop()
            else:
                # Simple proportional control
                correction = int(ALIGN_SPEED * abs(error_2d))
                
                if error_2d > 0:  # Marker is right of center
                    self.motors.send_command(MotorCommand(correction, -correction))
                else:  # Marker is left of center
                    self.motors.send_command(MotorCommand(-correction, correction))
    
    def _handle_approach(self, markers: List[MarkerData]):
        """Handle APPROACH state"""
        current_time = time.time()
        
        # Find target marker
        target_marker = next((m for m in markers if m.id == self.target_id), None)
        
        if not target_marker:
            # Lost marker
            if current_time - self.last_marker_time > LOST_TIMEOUT:
                DebugLogger.log_warning("Lost target during approach")
                self._change_state(NavState.SCAN_RIGHT, "Target lost")
                self.scan_start_time = current_time
            return
        
        self.last_marker_time = current_time
        
        # Check arrival conditions
        arrived_by_area = target_marker.area > STOP_AREA
        arrived_by_distance = self.motors.distance_cm < DISTANCE_STOP_CM
        
        if arrived_by_area or arrived_by_distance:
            reason = "Area threshold" if arrived_by_area else "Distance threshold"
            DebugLogger.log_success(f"Arrived! {reason}")
            self._change_state(NavState.ARRIVED, reason)
            self.motors.stop()
            return
        
        # Estimate 3D pose for guidance
        rvec, tvec, yaw, distance, success = self.pose_estimator.estimate_pose(target_marker.corners)
        
        if success:
            # Calculate steering correction based on yaw
            yaw_error = yaw / 45.0  # Normalize to [-1, 1] for ±45°
            
            # Apply correction while moving forward
            base_speed = APPROACH_SPEED
            correction = int(base_speed * yaw_error * 0.3)  # 30% correction factor
            
            left_speed = base_speed - correction
            right_speed = base_speed + correction
            
            self.motors.send_command(MotorCommand(left_speed, right_speed))
            
            if LOG_LEVEL == "DEBUG":
                DebugLogger.log_info(f"Approaching: L={left_speed}, R={right_speed}, Yaw={yaw:.1f}°")
        else:
            # Fallback: just move forward
            self.motors.send_command(MotorCommand(APPROACH_SPEED, APPROACH_SPEED))
    
    def emergency_stop(self):
        """Emergency stop"""
        DebugLogger.log_warning("EMERGENCY STOP")
        self._change_state(NavState.IDLE, "Emergency stop")
        self.target_id = None
        self.motors.stop()
    
    def run(self):
        """Main navigation loop"""
        DebugLogger.log_success("Starting debug navigation...")
        
        try:
            while True:
                # Get frame
                frame, frame_lag = self.video.get_frame()
                
                if frame is None:
                    time.sleep(0.1)
                    continue
                
                # Detect markers
                markers = self.detector.detect(frame)
                self.total_markers += len(markers)
                
                # Log marker detection
                DebugLogger.log_marker_detection(markers, self.target_id)
                
                # Log vision stats occasionally
                if time.time() % 2 < 0.1:  # Every ~2 seconds
                    DebugLogger.log_vision_stats(self.video.fps, frame_lag)
                
                # Check for hazards
                if any(m.id == LocationID.HAZARD.value for m in markers):
                    DebugLogger.log_error("HAZARD DETECTED!")
                    self.emergency_stop()
                
                # State machine (only in auto mode)
                if not self.manual_mode and self.target_id is not None:
                    if self.state == NavState.SCAN_RIGHT or self.state == NavState.SCAN_LEFT:
                        self._handle_scan(markers)
                    elif self.state == NavState.ALIGN:
                        self._handle_align(markers)
                    elif self.state == NavState.APPROACH:
                        self._handle_approach(markers)
                    elif self.state == NavState.ARRIVED:
                        # Wait a moment, then reset
                        time.sleep(2)
                        DebugLogger.log_success(f"Mission complete: {self.target_name}")
                        self.target_id = None
                        self._change_state(NavState.IDLE, "Mission complete")
                
                # Draw visual overlays
                self._draw_debug_overlay(frame, markers)
                
                # Show frame
                cv2.imshow("Debug Navigator", frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                self._handle_keyboard(key)
                
        except KeyboardInterrupt:
            DebugLogger.log_info("Interrupted by user")
        finally:
            self._cleanup()
    
    def _draw_debug_overlay(self, frame: np.ndarray, markers: List[MarkerData]):
        """Draw comprehensive debugging overlay"""
        
        # Draw marker detection
        self.detector.draw_detection_overlay(frame, markers, self.target_id)
        
        # Draw target marker with 3D axes if available
        if self.target_id:
            target_marker = next((m for m in markers if m.id == self.target_id), None)
            if target_marker:
                # Estimate and draw 3D pose
                rvec, tvec, _, _, success = self.pose_estimator.estimate_pose(target_marker.corners)
                if success:
                    self.pose_estimator.draw_3d_axes(frame, rvec, tvec)
        
        # Draw HUD
        self._draw_hud(frame)
    
    def _draw_hud(self, frame: np.ndarray):
        """Draw heads-up display"""
        # Background for text
        cv2.rectangle(frame, (0, 0), (IMAGE_WIDTH, 100), (0, 0, 0), -1)
        
        # State indicator (with color coding)
        state_colors = {
            NavState.IDLE: (200, 200, 200),
            NavState.SCAN_RIGHT: (255, 165, 0),
            NavState.SCAN_LEFT: (255, 165, 0),
            NavState.ALIGN: (255, 255, 0),
            NavState.APPROACH: (0, 255, 0),
            NavState.ARRIVED: (0, 255, 255),
            NavState.MANUAL: (255, 0, 255),
            NavState.ERROR: (0, 0, 255)
        }
        
        state_color = state_colors.get(self.state, (255, 255, 255))
        
        # Line 1: State
        cv2.putText(frame, f"State: {self.state.value}", (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_color, 2)
        
        # Line 2: Target
        target_text = f"Target: {self.target_name or 'None'}"
        cv2.putText(frame, target_text, (10, 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Line 3: Mode
        mode_text = f"Mode: {'MANUAL' if self.manual_mode else 'AUTO'}"
        cv2.putText(frame, mode_text, (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Line 4: Stats
        stats_text = f"FPS:{self.video.fps:2d} Markers:{self.total_markers}"
        cv2.putText(frame, stats_text, (IMAGE_WIDTH - 120, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw center line
        cv2.line(frame, (X_CENTER, 0), (X_CENTER, IMAGE_HEIGHT), (0, 0, 255), 1)
        
        # Draw alignment threshold lines
        threshold = int(X_CENTER * 0.1)  # 10% threshold
        cv2.line(frame, (X_CENTER - threshold, 0), 
                (X_CENTER - threshold, IMAGE_HEIGHT), (50, 50, 50), 1)
        cv2.line(frame, (X_CENTER + threshold, 0), 
                (X_CENTER + threshold, IMAGE_HEIGHT), (50, 50, 50), 1)
    
    def _handle_keyboard(self, key: int):
        """Handle keyboard input"""
        if key == ord('q'):
            DebugLogger.log_info("Quitting...")
            raise KeyboardInterrupt
        
        elif key == ord('s'):
            self.emergency_stop()
        
        elif key == ord('m'):
            self.manual_mode = not self.manual_mode
            mode = "MANUAL" if self.manual_mode else "AUTO"
            DebugLogger.log_info(f"Mode: {mode}")
            
            if self.manual_mode:
                self._change_state(NavState.MANUAL, "Manual mode enabled")
            else:
                self._change_state(NavState.IDLE, "Auto mode enabled")
        
        elif key == ord('k'):
            self.set_target(LocationID.KITCHEN)
        
        elif key == ord('b'):
            self.set_target(LocationID.BEDROOM)
        
        elif key == ord('h'):
            self.set_target(LocationID.HALLWAY)
        
        elif self.manual_mode:
            # Manual controls
            if key == ord('w'):  # Forward
                self.motors.send_command(MotorCommand(150, 150))
                DebugLogger.log_info("Manual: Forward")
            elif key == ord('s'):  # Backward
                self.motors.send_command(MotorCommand(-150, -150))
                DebugLogger.log_info("Manual: Backward")
            elif key == ord('a'):  # Left
                self.motors.send_command(MotorCommand(-100, 100))
                DebugLogger.log_info("Manual: Left")
            elif key == ord('d'):  # Right
                self.motors.send_command(MotorCommand(100, -100))
                DebugLogger.log_info("Manual: Right")
            elif key == 32:  # Space
                self.motors.stop()
                DebugLogger.log_info("Manual: Stop")
    
    def _cleanup(self):
        """Cleanup resources"""
        self.motors.stop()
        cv2.destroyAllWindows()
        
        # Print final statistics
        print("\n" + "="*70)
        print("📊 FINAL DEBUG STATISTICS")
        print("="*70)
        print(f"Total state changes: {self.state_changes}")
        print(f"Total markers detected: {self.total_markers}")
        print(f"Final state: {self.state.value}")
        print(f"Average FPS: {self.video.fps}")
        print("="*70)

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================
if __name__ == "__main__":
    # Clear screen
    os.system('cls' if os.name == 'nt' else 'clear')
    
    print("🤖 WHEELCHAIR DEBUG NAVIGATION SYSTEM")
    print("   Comprehensive Terminal Logging + Visual Debugging\n")
    
    # Check OpenCV
    try:
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    except:
        print("❌ OpenCV not properly installed")
        print("   Run: pip install opencv-contrib-python")
        exit(1)
    
    # Create and run navigator
    navigator = DebugNavigator()
    navigator.run()