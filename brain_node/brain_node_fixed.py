import cv2
import numpy as np
import websocket
import threading
import time
import json
from enum import Enum

# --- CONFIGURATION ---
PHONE_IP = "192.168.0.105" 
VISION_URL = f"http://192.168.0.105:8080/video"
ACTUATOR_WS = "ws://192.168.0.118/ws"

X_CENTER = 320         
STEER_DEADZONE = 60    
STOP_AREA = 160000 

class Mode(Enum):
    MANUAL = "MANUAL CONTROL"
    AUTO = "AUTONOMOUS MODE"

class BaseStation:
    def __init__(self):
        self.mode = Mode.MANUAL
        self.last_cmd = "S"
        self.ws = None
        
        # --- THREADED VIDEO ---
        self.cap = cv2.VideoCapture(VISION_URL, cv2.CAP_FFMPEG)
        self.ret, self.frame = False, None
        threading.Thread(target=self._video_worker, daemon=True).start()

        # --- ARUCO SETUP ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # --- ACTUATOR CONNECTION ---
        self._connect_actuator()
        threading.Thread(target=self._heartbeat_worker, daemon=True).start()

    def _video_worker(self):
        while True:
            self.ret, frame = self.cap.read()
            if self.ret:
                self.frame = cv2.resize(frame, (640, 480))

    def _connect_actuator(self):
        try:
            self.ws = websocket.create_connection(ACTUATOR_WS, timeout=2)
            print("✅ Actuator Connected")
        except: print("❌ Actuator Offline")

    def _heartbeat_worker(self):
        while True:
            if self.ws:
                try: self.ws.send(json.dumps({"cmd": "H"}))
                except: pass
            time.sleep(0.1)

    def send_move(self, direction, speed):
        if direction != self.last_cmd:
            try:
                cmd = json.dumps({"cmd": "M", "dir": direction, "spd": speed})
                self.ws.send(cmd)
                self.last_cmd = direction
                print(f"[{self.mode.name}] SENT: {direction} @ {speed}")
            except: pass

    def run(self):
        print(f"\n--- {self.mode.value} ---")
        print("M: Toggle Mode | W,A,S,D: Drive | SPACE: STOP | Q: Quit")

        while True:
            if self.frame is None: continue
            
            display_frame = self.frame.copy()
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            marker_info = "No Marker"

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                id_list = ids.flatten()
                
                if self.mode == Mode.AUTO:
                    if 0 in id_list:
                        self.send_move("S", 0)
                        marker_info = "!!! HAZARD !!!"
                    elif 9 in id_list:
                        idx = np.where(id_list == 9)[0][0]
                        mx = (corners[idx][0][0][0] + corners[idx][0][2][0]) / 2
                        area = cv2.contourArea(corners[idx])
                        marker_info = f"Target ID:9 | Area: {int(area)}"

                        if area > STOP_AREA:
                            self.send_move("S", 0)
                            marker_info = "ARRIVED"
                        elif mx < (X_CENTER - STEER_DEADZONE): self.send_move("L", 150)
                        elif mx > (X_CENTER + STEER_DEADZONE): self.send_move("R", 150)
                        else: self.send_move("F", 180)
                else:
                    if len(id_list) > 0:
                        marker_info = f"MANUAL - ID:{id_list[0]} Area:{int(cv2.contourArea(corners[0]))}"

            elif self.mode == Mode.AUTO:
                self.send_move("S", 0)
            
            # --- HUD ---
            cv2.rectangle(display_frame, (0, 0), (640, 65), (0,0,0), -1)
            mode_color = (0, 255, 0) if self.mode == Mode.AUTO else (0, 165, 255)
            cv2.putText(display_frame, f"MODE: {self.mode.value}", (20, 25), 1, 1.2, mode_color, 2)
            cv2.putText(display_frame, f"INFO: {marker_info}", (20, 50), 1, 1.0, (255, 255, 255), 1)

            cv2.imshow("Wheelchair Control Center", display_frame)
            
            # --- KEYBOARD CONTROLS ---
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'): 
                self.send_move("S", 0)
                break
            elif key == ord('m'): # Toggle Mode
                self.mode = Mode.AUTO if self.mode == Mode.MANUAL else Mode.MANUAL
                self.send_move("S", 0)
                print(f"Switched to {self.mode.value}")
            
            # --- MANUAL MODE COMMANDS ---
            if self.mode == Mode.MANUAL:
                if key == ord('w'):   self.send_move("F", 200) # Forward
                elif key == ord('s'): self.send_move("B", 150) # Backward (Changed from Stop)
                elif key == ord('a'): self.send_move("L", 160) # Left
                elif key == ord('d'): self.send_move("R", 160) # Right
                elif key == 32:       self.send_move("S", 0)   # SPACE BAR for STOP

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    station = BaseStation()
    station.run()