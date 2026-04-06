# 🤖 ASB — Autonomous Surveillance Bot

> A fully wireless, AI-powered surveillance robot built on dual ESP32s, with real-time YOLOv8 threat detection, IMU-based tilt compensation, an ultrasonic radar sweep, and a live browser dashboard.

---

> 🚧 **This project is actively maintained and open to contributions!**
> If you find a bug, have a suggestion, or want to improve something — please open an [issue](../../issues) or submit a [pull request](../../pulls). I'd really appreciate it!
> If you find this useful or interesting, consider giving it a ⭐ — it genuinely helps!

---

🍴 **Feel free to fork this repo and make it your own.**
Whether you want to swap the motors, add a different AI model, or build a completely new dashboard on top of it — go for it.
Pull requests are warmly welcomed, no matter how small. Even fixing a typo counts! 😊

## ✨ Features

- **Dual-mode operation** — Autonomous patrol with obstacle avoidance, or manual WASD/D-pad control
- **YOLOv8 threat detection** — Detects persons, cars, motorcycles, buses, and trucks in real time; draws colour-coded bounding boxes on the live feed
- **MPU-6050 IMU** — Pitch/roll monitoring with adaptive motor speed (climbs slopes faster, slows downhill), tip-over detection and auto-stop
- **Servo radar** — 180° ultrasonic sweep at 30 ms intervals; visualised as a live animated radar on the dashboard
- **MJPEG video stream** — ESP32-CAM streams 640×480 over HTTP; Python client overlays IMU HUD, mode badge, threat banner, and timestamp before forwarding to the browser
- **WebSocket architecture** — Bi-directional real-time comms between ESP32 ↔ Python ↔ Browser; ~100 ms IMU packets, ~30 ms radar packets, 2 s heartbeat
- **Tactical dashboard** — Orbitron/monospace HUD with radar panel, camera feed, IMU horizon indicator, tilt history graph, controls panel, and event log

---

## 📸 Dashboard Preview

> *<img width="1470" height="831" alt="image" src="https://github.com/user-attachments/assets/0e30e410-b6b1-4c67-a1e7-ee401bfffd10" />
*

---

## 🗂️ Repository Structure

```
asb/
├── firmware/
│   ├── esp32_main.ino        # Main ESP32 — motors, servo, ultrasonic, IMU, WebSocket server
│   └── esp32cam_stream.ino   # ESP32-CAM — MJPEG HTTP stream + snapshot endpoint
├── backend/
│   └── asb_client.py         # Python PC client — YOLO, OpenCV, Flask-SocketIO bridge
├── frontend/
│   └── dashboard.html        # Browser dashboard — radar, camera, IMU, controls, log
├── docs/
│   └── wiring_diagram.svg    # Full wiring reference
└── README.md
```

---

## 🛒 Hardware

| Component | Purpose |
|---|---|
| ESP32 Dev Module | Main controller — motors, servo, ultrasonic, IMU, WebSocket |
| ESP32-CAM (AI-Thinker) | Video streaming |
| MPU-6050 | IMU — pitch/roll/tilt detection |
| HC-SR04 | Ultrasonic distance sensor (radar sweep) |
| 2× TB6612FNG | Dual H-bridge motor drivers |
| 4× DC motors | Drivetrain |
| SG90 / MG90S servo | Radar sweep |
| 7.4V LiPo battery | Main power |
| Buck converter (7.4→5V) | Logic power rail |

---

## ⚡ System Architecture

```
[Battery 7.4V]
      │
   [Buck 5V]──────────────────────────────┐
      │                                   │
[ESP32 Main]                        [ESP32-CAM]
   │   ├──[TB6612 #1]──[Motor 1, 2]      │
   │   ├──[TB6612 #2]──[Motor 3, 4]      │ HTTP /stream
   │   ├──[Servo]──[HC-SR04]              │
   │   └──[MPU-6050 I2C]                  │
   │                                      │
   └──WiFi WebSocket──[PC Python Client]──┘
                           │
                    [Flask-SocketIO]
                           │
                    [Browser Dashboard]
               (Radar · Camera · IMU · Controls)
```

---

## 🚀 Quick Start

### 1 — Wiring

Follow `docs/wiring_diagram.svg`. Key points:
- **Common GND** — connect ALL grounds (ESP32, TB6612s, battery negative)
- **Motor power** — TB6612 VM pin = 7.4 V directly from battery
- **Logic power** — buck converter 7.4 V → 5 V → ESP32 VIN and TB6612 VCC
- **MPU-6050** — SDA → GPIO 21, SCL → GPIO 22
- **ESP32-CAM** — powered from the same 5 V rail, same WiFi network

### 2 — Arduino Libraries

Install via **Arduino IDE → Tools → Manage Libraries**:

```
ESPAsyncWebServer   (lacamera)
AsyncTCP            (dvarrel)
ESP32Servo          (Kevin Harrington)
NewPing             (Tim Eckel)
ArduinoJson         (Benoit Blanchon)
MPU6050             (Electronic Cats)
```

### 3 — Configure WiFi

Edit **both** `.ino` files:
```cpp
const char* SSID     = "YOUR_WIFI_SSID";
const char* PASSWORD = "YOUR_WIFI_PASSWORD";
```

### 4 — Upload Firmware

| File | Target board | Partition |
|---|---|---|
| `esp32_main.ino` | ESP32 Dev Module | Default |
| `esp32cam_stream.ino` | AI Thinker ESP32-CAM | Huge App (3MB No OTA) |

Open Serial Monitor at **115200 baud** after power-on — note both IP addresses.

### 5 — Python Setup

```bash
pip install opencv-python ultralytics websockets flask flask-socketio numpy requests
```

YOLOv8n weights (~6 MB) download automatically on first run.

### 6 — Run

```bash
python backend/asb_client.py --esp32 192.168.x.xxx --cam 192.168.x.yyy
```

Open **http://localhost:5000** in your browser.

---

## 🎮 Controls

| Action | Input |
|---|---|
| Switch to Control mode | Click **CONTROL** button |
| Drive | `W A S D` or arrow keys, or D-pad |
| Emergency stop | `SPACE` |
| Speed adjust | Slider in Controls panel |
| Switch to Surveillance | Click **SURVEILLANCE** button |

---

## 🔍 Threat Detection

YOLOv8 runs continuously and highlights:

| Class | Box colour |
|---|---|
| Person | 🔴 Red |
| Car / Motorcycle / Bus / Truck | 🟠 Orange |
| Other objects | 🟢 Green |

To add fire detection, download a fire model from [Roboflow Universe](https://universe.roboflow.com/search?q=fire+detection) and update `YOLO_MODEL` in `asb_client.py`.

---

## 📡 IMU & Slope Compensation

The MPU-6050 updates every 100 ms. Motor speed adapts to terrain automatically:

```
factor = 1.0 + (pitch / 90) × 0.6
Uphill  (+30°) → factor 1.20 → base speed 192
Downhill (−20°) → factor 0.87 → base speed 139
Clamped: [70, 245]
```

If the bot tips over (|pitch| or |roll| > 45°), motors stop and an alert LED fires.

---

## 🔧 Troubleshooting

| Symptom | Fix |
|---|---|
| ESP32 won't connect to WiFi | Verify SSID/password; ESP32 requires 2.4 GHz (no 5 GHz) |
| Camera feed blank | Check ESP32-CAM IP in Serial Monitor; confirm same network |
| Motors not moving | Verify STBY1 (GPIO 33) and STBY2 (GPIO 32) are HIGH; check common GND |
| WebSocket disconnect loop | Check firewall; ESP32 and PC must be on the same subnet |
| YOLO slow / frame drops | Use `yolov8n.pt`; reduce `FRAME_W` to 320 in `asb_client.py` |
| IMU "NOT FOUND" on boot | Check SDA/SCL wiring; confirm AD0 → GND (address 0x68) |

---

## 🛠️ Built With

![ESP32](https://img.shields.io/badge/ESP32-Espressif-red?style=flat-square)
![Python](https://img.shields.io/badge/Python-3.10+-blue?style=flat-square&logo=python)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-purple?style=flat-square)
![OpenCV](https://img.shields.io/badge/OpenCV-4-green?style=flat-square&logo=opencv)
![Flask](https://img.shields.io/badge/Flask-SocketIO-black?style=flat-square&logo=flask)

---

## 📄 License

MIT — see [LICENSE](LICENSE) for details.
