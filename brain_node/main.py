import cv2
import numpy as np
import websocket
import threading
import time
import json
import os
import sys
import signal
import urllib.request
from enum import Enum

# --- 1. CONFIGURATION ---
VISION_URL = "http://192.168.0.126/"      
ACTUATOR_WS = "ws://192.168.0.118/ws"    

# Tuning Constants (Calibration Mode helps find these)
X_CENTER = 160          
STEER_DEADZONE = 35     
STOP_AREA = 18000       

# Speed Settings
SPD_SEARCH = 100
SPD_ALIGN = 120
SPD_FORWARD = 180
SPD_BACK = 140

os.environ['OPENCV_LOG_LEVEL'] = 'OFF'

# --- EMERGENCY SIGNAL HANDLER ---
def signal_handler(sig, frame):
    print("\n⚠️ Emergency shutdown triggered!")
    try:
        temp_ws = websocket.create_connection(ACTUATOR_WS, timeout=1)
        temp_ws.send(json.dumps({"cmd": "M", "dir": "S", "spd": 0}))
        temp_ws.close()
    except: pass
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Mode(Enum):
    MANUAL = "MANUAL CONTROL"
    AUTO = "AUTONOMOUS MODE"

class VideoStream:
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.ret = False
        self.running = True
        try:
            urllib.request.urlopen(url, timeout=3)
            print("✅ Camera Node Linked")
            threading.Thread(target=self.update, daemon=True).start()
        except:
            print("❌ Camera Node Offline")
            self.running = False

    def update(self):
        try:
            stream = urllib.request.urlopen(self.url)
            bytes_data = b''
            while self.running:
                bytes_data += stream.read(2048)
                a = bytes_data.find(b'\xff\xd8')
                b = bytes_data.find(b'\xff\xd9')
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
        self.target_id = 9
        self.last_cmd = "S"
        self.search_dir = "R"
        self.conf_counter = 0    
        self.lost_counter = 0
        self.target_locked = False 
        
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters()
        )
        
        self.connect_actuator()
        threading.Thread(target=self.heartbeat, daemon=True).start()

    def connect_actuator(self):
        try:
            self.ws = websocket.create_connection(ACTUATOR_WS, timeout=2)
            print("✅ Actuator Node Linked")
        except: print("❌ Actuator Offline")

    def heartbeat(self):
        count = 0
        while True:
            if self.ws:
                try: 
                    self.ws.send(json.dumps({"cmd": "H", "cnt": count}))
                    count += 1
                    if count % 10 == 0: self.ws.ping() 
                except:
                    print("⚠️ WS Connection lost. Retrying...")
                    self.connect_actuator()
            time.sleep(0.1)

    def send_move(self, direction, speed):
        if direction != self.last_cmd:
            try:
                cmd = json.dumps({"cmd": "M", "dir": direction, "spd": speed})
                self.ws.send(cmd)
                self.last_cmd = direction
                print(f"[{self.mode.name}] -> {direction} @ {speed}")
            except: pass

    def calibrate_area(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None and self.target_id in ids.flatten():
            id_list = list(ids.flatten())
            idx = id_list.index(self.target_id)
            area = cv2.contourArea(corners[idx])
            return int(area)
        return None

    def process_logic(self, frame, has_video):
        status = "NO VIDEO"
        marker_data = ""
        if not has_video: return frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        id_list = list(ids.flatten()) if ids is not None else []
        
        # --- CONFIDENCE FILTERING ---
        if self.target_id in id_list:
            self.conf_counter += 1; self.lost_counter = 0
            if self.conf_counter >= 3: self.target_locked = True
        else:
            self.lost_counter += 1; self.conf_counter = 0
            if self.lost_counter >= 5: self.target_locked = False

        if ids is not None: cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # --- DECISION TREE ---
        if self.mode == Mode.AUTO:
            if 0 in id_list:
                self.send_move("S", 0); status = "!!! HAZARD !!!"
            elif self.target_locked and (self.target_id in id_list):
                idx = id_list.index(self.target_id)
                mx = (corners[idx][0][0][0] + corners[idx][0][2][0]) / 2
                area = cv2.contourArea(corners[idx])
                marker_data = f"AREA: {int(area)}"
                if area > STOP_AREA:
                    self.send_move("S", 0); status = "ARRIVED"
                elif mx < (X_CENTER - STEER_DEADZONE):
                    self.send_move("L", SPD_ALIGN); self.search_dir = "L"; status = "ALIGN LEFT"
                elif mx > (X_CENTER + STEER_DEADZONE):
                    self.send_move("R", SPD_ALIGN); self.search_dir = "R"; status = "ALIGN RIGHT"
                else:
                    self.send_move("F", SPD_FORWARD); status = "FORWARD"
            elif 3 in id_list:
                self.send_move("F", 150); status = "PATH: HALLWAY"
            else:
                self.send_move(self.search_dir, SPD_SEARCH); status = f"SEARCHING {self.search_dir}"
        else:
            status = "MANUAL MODE"

        # HUD
        cv2.rectangle(frame, (0,0), (320, 55), (0,0,0), -1)
        mode_color = (0, 255, 0) if self.mode == Mode.AUTO else (0, 165, 255)
        cv2.putText(frame, self.mode.value, (10, 20), 1, 0.8, mode_color, 1)
        cv2.putText(frame, f"{status} {marker_data}", (10, 42), 1, 0.7, (255,255,255), 1)
        return frame

def main():
    vs = VideoStream(VISION_URL)
    brain = SmartBrain()
    calib_mode = False
    
    print("\n" + "="*40 + "\nMISSION CONTROL ACTIVE\n" + "="*40)
    print("M: Toggle Mode | C: Calibrate | SPACE: Stop")

    while True:
        success, frame = vs.read()
        
        if calib_mode:
            area = brain.calibrate_area(frame)
            if area:
                cv2.putText(frame, f"CALIBRATING: {area}", (10, 80), 1, 1, (0, 255, 255), 2)
                global STOP_AREA
                STOP_AREA = area

        debug_frame = brain.process_logic(frame, success)
        cv2.imshow("Wheelchair Control Center", debug_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('m'):
            brain.mode = Mode.AUTO if brain.mode == Mode.MANUAL else Mode.MANUAL
            brain.send_move("S", 0)
        elif key == ord('c') and brain.mode == Mode.MANUAL:
            calib_mode = not calib_mode
            print(f"Calibration {'Active' if calib_mode else 'Saved'}")
        elif brain.mode == Mode.MANUAL:
            if key == ord('w'):   brain.send_move("F", 200)
            elif key == ord('s'): brain.send_move("B", SPD_BACK)
            elif key == ord('a'): brain.send_move("L", 160)
            elif key == ord('d'): brain.send_move("R", 160)
            elif key == 32:       brain.send_move("S", 0)
        
        if key == ord('q'): break

    vs.stop(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()