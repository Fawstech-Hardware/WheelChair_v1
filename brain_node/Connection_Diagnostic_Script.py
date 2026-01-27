import cv2
import websocket
import json
import time

# --- YOUR IPs ---
VISION_URL = "http://192.168.0.126"
ACTUATOR_WS = "ws://192.168.0.118/ws"

print("--- DIAGNOSTIC START ---")

# 1. TEST ACTUATOR (WebSocket)
print(f"1. Testing Actuator at {ACTUATOR_WS}...")
try:
    ws = websocket.create_connection(ACTUATOR_WS, timeout=3)
    print("   ✅ SUCCESS: WebSocket Connected!")
    print("   Sending test command: STOP")
    ws.send(json.dumps({"cmd": "M", "dir": "S", "spd": 0}))
    ws.close()
except Exception as e:
    print(f"   ❌ FAILED: Actuator unreachable. Error: {e}")

# 2. TEST VISION (OpenCV)
print(f"2. Testing Vision at {VISION_URL}...")
cap = cv2.VideoCapture(VISION_URL)
if cap.isOpened():
    print("   ✅ SUCCESS: Camera stream opened!")
    ret, frame = cap.read()
    if ret:
        print("   ✅ SUCCESS: Frame captured! Closing window in 3 seconds.")
        cv2.imshow("DIAGNOSTIC", frame)
        cv2.waitKey(3000)
    else:
        print("   ❌ FAILED: Stream opened but no images received.")
    cap.release()
else:
    print("   ❌ FAILED: Could not open camera stream. Is a browser tab still open?")

print("--- DIAGNOSTIC END ---")
cv2.destroyAllWindows()