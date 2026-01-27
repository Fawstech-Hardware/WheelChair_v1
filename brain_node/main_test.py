import cv2
import numpy as np
import websocket
import threading
import time
import json
import os
import urllib.request
from enum import Enum

# --- 1. CONFIGURATION ---
VISION_URL = "http://192.168.0.126/"      
ACTUATOR_WS = "ws://192.168.0.118/ws"    

X_CENTER = 160          
STEER_DEADZONE = 35     
STOP_AREA = 18000       

# Speed Settings
SPD_MAX_AUTO = 180      
SPD_MIN_AUTO = 125      
SPD_KICKSTART = 230     
SPD_ALIGN = 140         
SPD_SEARCH = 110        

os.environ['OPENCV_LOG_LEVEL'] = 'OFF'

class Mode(Enum):
    MANUAL = "MANUAL CONTROL"
    AUTO = "AUTONOMOUS MISSION"

class VideoStream:
    """Manual Byte-Stream Reader: Fixes the Black Window Issue"""
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.ret = False
        self.running = True
        try:
            # Check if reachable
            urllib.request.urlopen(url, timeout=3)
            print("✅ Vision Node Linked")
            threading.Thread(target=self.update, daemon=True).start()
        except Exception as e:
            print(f"❌ Vision Node Offline: {e}")
            self.running = False

    def update(self):
        try:
            stream = urllib.request.urlopen(self.url)
            bytes_data = b''
            while self.running:
                bytes_data += stream.read(2048)
                a = bytes_data.find(b'\xff\xd8') # JPEG Start
                b = bytes_data.find(b'\xff\xd9') # JPEG End
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    if len(jpg) > 100:
                        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            self.frame = img
                            self.ret = True
        except: self.ret = False

    def read(self):
        if self.ret and self.frame is not None: return True, self.frame
        return False, np.zeros((240, 320, 3), dtype=np.uint8)

class SmartBrain:
    def __init__(self):
        self.ws = None
        self.mode = Mode.MANUAL
        self.target_id = None  
        self.last_cmd = "S"
        self.search_dir = "R"
        self.conf_counter = 0    
        self.lost_counter = 0    
        self.target_locked = False 
        self.min_move_time = 0.4  
        self.last_action_time = 0
        self.current_actuation = "S"
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        
        self.connect_actuator()
        threading.Thread(target=self.heartbeat, daemon=True).start()

    def connect_actuator(self):
        try:
            self.ws = websocket.create_connection(ACTUATOR_WS, timeout=2)
            print("✅ Actuator Node Linked")
        except: print("❌ Actuator Offline")

    def heartbeat(self):
        while True:
            if self.ws:
                try: self.ws.send(json.dumps({"cmd": "H"}))
                except: pass
            time.sleep(0.1)

    def send_move(self, direction, speed):
        now = time.time()
        final_speed = speed
        if self.current_actuation == "S" and direction != "S":
            final_speed = SPD_KICKSTART # Apply Kickstart torque

        if direction != self.last_cmd:
            if (now - self.last_action_time) < self.min_move_time:
                return 

            try:
                cmd = json.dumps({"cmd": "M", "dir": direction, "spd": final_speed})
                self.ws.send(cmd)
                self.last_cmd = direction
                self.current_actuation = direction
                self.last_action_time = now
                print(f"ACTION: {direction} @ {final_speed}")
            except: pass

    def calculate_scaled_speed(self, area):
        if area >= STOP_AREA: return 0
        dist_factor = 1.0 - (area / STOP_AREA)
        speed = SPD_MIN_AUTO + (dist_factor * (SPD_MAX_AUTO - SPD_MIN_AUTO))
        return int(speed)

    def process_logic(self, frame, success):
        status = "IDLE"
        marker_info = "Waiting for Target (1,2,3)"
        if not success: return frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        id_list = list(ids.flatten()) if ids is not None else []
        
        if ids is not None: cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if self.mode == Mode.AUTO:
            if self.target_id is None:
                status = "SELECT ROOM (1, 2, 3)"
                self.send_move("S", 0)
            elif 0 in id_list:
                status = "!!! HAZARD DETECTED !!!"
                self.send_move("S", 0)
            else:
                if self.target_id in id_list:
                    self.conf_counter += 1
                    self.lost_counter = 0
                    if self.conf_counter >= 2: self.target_locked = True
                else:
                    self.lost_counter += 1; self.conf_counter = 0
                    if self.lost_counter >= 4: self.target_locked = False

                if self.target_locked and (self.target_id in id_list):
                    idx = id_list.index(self.target_id)
                    mx = (corners[idx][0][0][0] + corners[idx][0][2][0]) / 2
                    area = cv2.contourArea(corners[idx])
                    speed = self.calculate_scaled_speed(area)
                    marker_info = f"ID:{self.target_id} | Area:{int(area)}"

                    if area > STOP_AREA:
                        status = "ARRIVED"
                        self.send_move("S", 0); self.target_id = None
                    elif mx < (X_CENTER - STEER_DEADZONE):
                        status = "ALIGNING LEFT"
                        self.send_move("L", SPD_ALIGN); self.search_dir = "L"
                    elif mx > (X_CENTER + STEER_DEADZONE):
                        status = "ALIGNING RIGHT"
                        self.send_move("R", SPD_ALIGN); self.search_dir = "R"
                    else:
                        status = "MOVING FORWARD"
                        self.send_move("F", speed)
                else:
                    status = f"SEARCHING {self.target_id}"
                    self.send_move(self.search_dir, SPD_SEARCH)
        
        else: # Manual Mode Info
            if ids is not None and len(id_list) > 0:
                status = f"Visible ID: {id_list[0]}"

        # --- UI DISPLAY ---
        cv2.rectangle(frame, (0,0), (320, 50), (0,0,0), -1)
        mode_color = (0, 255, 0) if self.mode == Mode.AUTO else (0, 165, 255)
        cv2.putText(frame, self.mode.value, (10, 20), 1, 0.8, mode_color, 1)
        cv2.putText(frame, f"{status}", (10, 38), 1, 0.7, (255,255,255), 1)
        cv2.putText(frame, marker_info, (10, 230), 1, 0.6, (0, 255, 255), 1)
        return frame

def main():
    vs = VideoStream(VISION_URL)
    brain = SmartBrain()

    while True:
        success, frame = vs.read()
        debug_frame = brain.process_logic(frame, success)
        cv2.imshow("Wheelchair Dashboard v4.4.1", debug_frame)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('1'): brain.target_id = 9; print("🎯 Target: KITCHEN")
        elif key == ord('2'): brain.target_id = 5; print("🎯 Target: BEDROOM")
        elif key == ord('3'): brain.target_id = 3; print("🎯 Target: HALLWAY")
        elif key == ord('m'):
            brain.mode = Mode.AUTO if brain.mode == Mode.MANUAL else Mode.MANUAL
            brain.send_move("S", 0)
        elif brain.mode == Mode.MANUAL:
            if key == ord('w'):   brain.send_move("F", 200)
            elif key == ord('s'): brain.send_move("B", 160)
            elif key == ord('a'): brain.send_move("L", 160)
            elif key == ord('d'): brain.send_move("R", 160)
            elif key == 32:       brain.send_move("S", 0)
        
        if key == ord('q'): break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()