import websocket
import json
import time

# Use your Actuator IP
ws_url = "ws://192.168.0.118/ws"

try:
    print("Connecting to Motors...")
    ws = websocket.create_connection(ws_url, timeout=5)
    print("✅ Connected! Moving Forward for 1 second...")
    ws.send(json.dumps({"cmd": "M", "dir": "F", "spd": 200}))
    time.sleep(1)
    ws.send(json.dumps({"cmd": "M", "dir": "S", "spd": 0}))
    print("Stopped.")
    ws.close()
except Exception as e:
    print(f"❌ Failed: {e}")