# 🦽 Autonomous Smart Wheelchair System

> **AI-Powered Indoor Navigation for Accessibility**

An autonomous wheelchair navigation system that uses computer vision, voice commands, and ArUco marker-based localization for safe indoor mobility.

---

## 🎯 What Does This System Do?

This wheelchair can **navigate autonomously** between rooms in your home or facility. Simply say "Go to kitchen" or press a button, and the wheelchair will:

1. 🔍 **Scan** the environment for navigation markers
2. 🎯 **Align** itself with the correct path
3. 🚀 **Navigate** to your destination avoiding obstacles
4. ✅ **Announce** arrival at the destination

---

## 🏗️ System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    🦽 SMART WHEELCHAIR                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌─────────────┐    WiFi    ┌─────────────┐                   │
│   │  📷 Vision  │ ─────────► │  🧠 Brain   │ ◄── 🎤 Voice      │
│   │    Node     │   Stream   │    Node     │     Commands      │
│   │  (Camera)   │            │  (AI/PC)    │                   │
│   └─────────────┘            └──────┬──────┘                   │
│                                     │                           │
│                                     │ WebSocket                 │
│                                     ▼                           │
│                              ┌─────────────┐                   │
│                              │  ⚙️ Actuator │ ──► 🔊 Buzzer    │
│                              │    Node     │                   │
│                              │  (Motors)   │ ──► 📏 Ultrasonic │
│                              └─────────────┘                   │
│                                     │                           │
│                          ┌─────────┴─────────┐                 │
│                          ▼                   ▼                 │
│                     [Left Motor]       [Right Motor]           │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 Hardware Required

### Components Checklist

| # | Component | Purpose | Quantity |
|---|-----------|---------|----------|
| 1 | Seeed XIAO ESP32-S3 Sense | Vision (Camera) | 1 |
| 2 | ESP32 DevKit | Motor Control | 1 |
| 3 | L298N Motor Driver | H-Bridge for DC Motors | 1 |
| 4 | DC Gear Motors (12V) | Wheelchair Drive | 2 |
| 5 | HC-SR04 Ultrasonic | Obstacle Detection | 1 |
| 6 | ArUco Markers (4x4) | Navigation Waypoints | Print from PDF |
| 7 | Windows PC / Raspberry Pi | Brain Node | 1 |
| 8 | USB Microphone | Voice Commands | 1 |
| 9 | Power Supply (12V, 5A+) | Motors & Electronics | 1 |

---

## 🗺️ Setting Up Navigation Markers

Print these **ArUco markers** (4x4_50 dictionary) and place them at locations:

| ArUco ID | Location | Placement |
|----------|----------|-----------|
| **3** | Bedroom | Door frame or wall |
| **5** | Bathroom | Door frame |
| **6** | Hallway | Central visible spot |
| **7** | Living Room | Wall or furniture |
| **9** | Kitchen | Door or cabinet |
| **0** | ⚠️ HAZARD | Any danger zone |

> 💡 **Tip:** Print markers at **10cm × 10cm** size. Place at camera height (~1 meter from floor).

### Generate Markers
Use Python to generate markers:
```python
import cv2
aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for id in [0, 3, 5, 6, 7, 9]:
    img = cv2.aruco.generateImageMarker(aruco, id, 200)
    cv2.imwrite(f"marker_{id}.png", img)
```

---

## ⚡ Quick Start Guide

### Step 1: Flash the Vision Node

```bash
cd vision_node
# Edit src/main.cpp - Set your WiFi credentials
pio run --target upload
```

> 📷 Verify: Open `http://172.20.11.126/` in browser to see camera stream.

---

### Step 2: Flash the Actuator Node

```bash
cd actuator_node
# Edit src/main.cpp - Set your WiFi credentials
pio run --target upload
```

> ⚙️ Verify: Serial monitor should show `ACTUATOR HAL v5.0 ONLINE`

---

### Step 3: Install Brain Node Dependencies

```bash
cd brain_node
pip install opencv-python numpy websocket-client pyttsx3 SpeechRecognition
```

---

### Step 4: Configure Network

Edit IP addresses in `brain_node/main.py`:

```python
VISION_URL = "http://172.20.11.126/"      # Vision Node IP
ACTUATOR_WS = "ws://172.20.11.118/ws"     # Actuator Node IP
```

> 🌐 All devices must be on the **same WiFi network**.

---

### Step 5: Run the System

```bash
cd brain_node
python main.py
```

---

## 🎮 How to Use

### Voice Commands
| Say This | Wheelchair Action |
|----------|-------------------|
| "Go to kitchen" | Navigate to kitchen |
| "Take me to bedroom" | Navigate to bedroom |
| "Stop" | Emergency stop |

### Keyboard Controls
| Key | Action |
|-----|--------|
| `K` | Go to Kitchen |
| `B` | Go to Bedroom |
| `H` | Go to Hallway |
| `L` | Go to Living Room |
| `T` | Go to Bathroom |
| `M` | Toggle Manual Mode |
| `Q` | Quit |

### Manual Mode (Press `M` first)
| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `Space` | Stop |

---

## 🔌 Wiring Guide

### Actuator Node Connections

```
ESP32 GPIO    →    Component
────────────────────────────
GPIO 4        →    L298N IN1 (Left Forward)
GPIO 2        →    L298N IN2 (Left Backward)
GPIO 14       →    L298N ENA (Left Speed PWM)
GPIO 32       →    L298N IN3 (Right Forward)
GPIO 33       →    L298N IN4 (Right Backward)
GPIO 27       →    L298N ENB (Right Speed PWM)
GPIO 5        →    HC-SR04 TRIG
GPIO 34       →    HC-SR04 ECHO
GPIO 12       →    Buzzer (+)
GPIO 13       →    Status LED (+)
GND           →    Common Ground
5V            →    Logic Power
```

---

## 🛡️ Safety Features

| Feature | Description |
|---------|-------------|
| **Obstacle Stop** | Stops 25cm before obstacles |
| **Hazard Markers** | Emergency stop on ArUco ID 0 |
| **Heartbeat Monitor** | Stops if brain connection lost |
| **Video Watchdog** | Stops if camera feed lost |
| **Buzzer Alert** | Beeps near obstacles |

---

## 📁 Project Structure

```
Wheelchair_System/
├── brain_node/           # 🧠 AI Controller (Python)
│   ├── main.py           # Main navigation logic
│   └── README.md         # Brain node documentation
│
├── actuator_node/        # ⚙️ Motor Controller (ESP32)
│   ├── src/main.cpp      # Firmware
│   └── README.md         # Actuator documentation
│
├── vision_node/          # 📷 Camera Module (ESP32-S3)
│   ├── src/main.cpp      # Firmware
│   └── README.md         # Vision documentation
│
└── README.md             # This file
```

---

## � Switching to Smartphone Hotspot

If you need to use a mobile hotspot instead of a router, follow these steps:

### 1. Configure Smartphone Hotspot
- **SSID (Name):** `Fawstech R&D`
- **Password:** `R&D@Fawstech`
- **Band:** **2.4 GHz** (Important: ESP32 does not support 5GHz)

### 2. Update Microcontroller Code (Vision & Actuator)
Smartphones usually assign dynamic IPs (e.g., `192.168.43.x`), so you must **disable static IP** configuration.

**For `vision_node/src/main.cpp` AND `actuator_node/src/main.cpp`:**

1. **Comment out** the static IP configuration:
   ```cpp
   // IPAddress local_IP(172, 20, 11, ...);
   // IPAddress gateway(172, 20, 11, 1);
   // IPAddress subnet(255, 255, 255, 0);
   ```
2. **Comment out** the `WiFi.config` line inside `setup()`:
   ```cpp
   // if (!WiFi.config(local_IP, gateway, subnet)) { ... }
   ```
3. **Upload** the code to both ESP32 boards.
4. OPEN **Serial Monitor** (baud 115200) to see the new assigned IPs (e.g., `192.168.43.50`).

### 3. Update Brain Node
Update `brain_node/main.py` (and `main_test.py`) with the **NEW IPs** shown in the Serial Monitor:

```python
VISION_URL = "http://192.168.43.XXX/"      # Replace XXX with Vision Node IP
ACTUATOR_WS = "ws://192.168.43.YYY/ws"     # Replace YYY with Actuator Node IP
```

---

## �🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| Camera not connecting | Check Vision Node IP, verify WiFi |
| Motors not moving | Check L298N wiring, power supply |
| Voice not recognized | Check microphone, ambient noise |
| Markers not detected | Improve lighting, marker size |
| Wheelchair spins in circles | Check motor wiring polarity |
| Emergency stops randomly | Check ultrasonic sensor wiring |

---

## 🌐 Network Diagram

```
                    WiFi Router
                    (Gateway: 172.20.11.1)
                         │
         ┌───────────────┼───────────────┐
         │               │               │
         ▼               ▼               ▼
   ┌──────────┐   ┌──────────┐   ┌──────────┐
   │  Vision  │   │  Brain   │   │ Actuator │
   │  Node    │   │  Node    │   │   Node   │
   │.126      │   │  (DHCP)  │   │  .118    │
   └──────────┘   └──────────┘   └──────────┘
```

---

## 📄 License

This project is developed by **Fawstech R&D** for accessibility solutions.

---

## 📞 Support

For technical support or customization requests, refer to individual node README files:
- [Brain Node Documentation](brain_node/README.md)
- [Actuator Node Documentation](actuator_node/README.md)
- [Vision Node Documentation](vision_node/README.md)

---

<div align="center">

**Built with ❤️ for Accessibility**

*Making mobility autonomous, one marker at a time.*

</div>
