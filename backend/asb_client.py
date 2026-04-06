"""
ASB — Autonomous Surveillance Bot  v2.0
Python PC Client  (IMU Edition)

New in v2:
  - Receives and relays IMU packets (pitch, roll, upright, slope, baseSpeed)
  - Tilt / tip-over alerts forwarded to browser dashboard
  - IMU history ring buffer for dashboard graphs

Requirements:
  pip install opencv-python ultralytics websockets flask flask-socketio numpy requests

Run:
  python asb_client.py --esp32 <ESP32_IP> --cam <CAM_IP>
"""

import asyncio
import base64
import argparse
import json
import logging
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import websockets
from flask import Flask, jsonify
from flask_socketio import SocketIO, emit
from ultralytics import YOLO

# ─── Logging ──────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("ASB")

# ─── Config ───────────────────────────────────────────────────
FLASK_PORT       = 5000
CAM_STREAM_PATH  = "/stream"
YOLO_MODEL       = "yolov8n.pt"
CONFIDENCE       = 0.45
FRAME_W, FRAME_H = 640, 480

THREAT_CLASSES = {
    0: ("person",     (0,   0, 255)),
    2: ("car",        (0, 165, 255)),
    3: ("motorcycle", (0, 165, 255)),
    5: ("bus",        (0, 165, 255)),
    7: ("truck",      (0, 165, 255)),
}

# ─── Shared State ─────────────────────────────────────────────
state = {
    "mode":          "surveillance",
    "connected_esp": False,
    "threat":        False,
    "threat_labels": [],
    "fps":           0.0,
    "detections":    [],
    "log":           deque(maxlen=60),

    # IMU
    "pitch":         0.0,
    "roll":          0.0,
    "upright":       True,
    "onSlope":       False,
    "slopeAngle":    0.0,
    "baseSpeed":     160,
    "imuFactor":     1.0,
    "imu_history":   deque(maxlen=60),   # 60 samples  ~6 s at 100 ms
}

esp32_ws      = None
latest_b64    = None
frame_lock    = threading.Lock()
ws_loop       = None   # assigned after asyncio loop creation

# ─── Flask + SocketIO ─────────────────────────────────────────
app = Flask(__name__)
app.config["SECRET_KEY"] = "asb_v2"
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


# ─── YOLO ─────────────────────────────────────────────────────
def load_model():
    log.info(f"Loading {YOLO_MODEL}…")
    m = YOLO(YOLO_MODEL)
    log.info("YOLO ready ✓")
    return m


# ─── Detection ────────────────────────────────────────────────
def detect(frame, model):
    results = model(frame, conf=CONFIDENCE, verbose=False)[0]
    detections, threat, labels = [], False, []

    for box in results.boxes:
        cls   = int(box.cls[0])
        conf  = float(box.conf[0])
        x1,y1,x2,y2 = map(int, box.xyxy[0])
        name  = model.names[cls]
        color = (0, 200, 50)
        is_threat = False

        if cls in THREAT_CLASSES:
            name, color = THREAT_CLASSES[cls]
            is_threat = threat = True
            labels.append(name)

        cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
        tag = f"{name} {conf:.0%}"
        (tw,th),_ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        cv2.rectangle(frame, (x1, y1-th-6), (x1+tw+4, y1), color, -1)
        cv2.putText(frame, tag, (x1+2, y1-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255,255,255), 1)

        detections.append({"label": name, "confidence": round(conf,2),
                           "bbox": [x1,y1,x2,y2], "threat": is_threat})
    return frame, threat, list(set(labels)), detections


# ─── IMU overlay on frame ─────────────────────────────────────
def draw_imu_overlay(frame):
    pitch = state["pitch"]
    roll  = state["roll"]
    upright  = state["upright"]
    on_slope = state["onSlope"]
    spd      = state["baseSpeed"]

    # Horizon line — shifts with roll, tilts with pitch
    cx, cy = FRAME_W // 2, FRAME_H // 2
    angle_rad = -roll * np.pi / 180
    length = 120
    dx = int(length * np.cos(angle_rad))
    dy = int(length * np.sin(angle_rad))
    pitch_offset = int(pitch * 1.5)

    # Artificial horizon bar
    color = (0,200,50) if upright else (0,0,220)
    cv2.line(frame,
             (cx - dx, cy + dy + pitch_offset),
             (cx + dx, cy - dy + pitch_offset),
             color, 2)

    # Centre cross
    cv2.line(frame, (cx-12, cy), (cx+12, cy), (180,180,180), 1)
    cv2.line(frame, (cx, cy-12), (cx, cy+12), (180,180,180), 1)

    # IMU readout — bottom-left corner
    y0 = FRAME_H - 80
    lines = [
        (f"PITCH: {pitch:+.1f}°",  (0,200,50)  if abs(pitch)<15 else (0,165,255)),
        (f"ROLL:  {roll:+.1f}°",   (0,200,50)  if abs(roll)<15  else (0,165,255)),
        (f"SPD:   {spd}",          (0,200,50)),
        (f"{'UPRIGHT' if upright else '⚠ TIPPED!'}",
                                   (0,200,50)  if upright       else (0,0,220)),
    ]
    for i,(txt,col) in enumerate(lines):
        cv2.putText(frame, txt, (10, y0 + i*18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1)

    # Slope indicator bar  (right edge)
    bar_h   = 100
    bar_x   = FRAME_W - 22
    bar_top = cy - bar_h//2
    cv2.rectangle(frame, (bar_x, bar_top), (bar_x+10, bar_top+bar_h), (40,40,40), -1)
    fill = int(bar_h * (0.5 + pitch/90.0))
    fill = max(0, min(bar_h, fill))
    fill_color = (0,165,255) if pitch>8 else ((0,200,50) if abs(pitch)<8 else (100,100,255))
    cv2.rectangle(frame,
                  (bar_x, bar_top + bar_h - fill),
                  (bar_x+10, bar_top+bar_h),
                  fill_color, -1)
    cv2.putText(frame, "SLP", (bar_x-2, bar_top-6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (140,140,140), 1)

    return frame


# ─── Camera thread ────────────────────────────────────────────
def camera_thread(cam_ip: str, model):
    global latest_b64, state

    url = f"http://{cam_ip}{CAM_STREAM_PATH}"
    log.info(f"Camera: {url}")

    fps_cnt, fps_t = 0, time.time()

    while True:
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            log.warning("Camera unavailable, retry in 3 s…")
            time.sleep(3)
            continue
        log.info("Camera stream connected ✓")

        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("Frame loss — reconnecting…")
                break

            frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            # YOLO detection
            frame, threat, labels, detections = detect(frame, model)

            # IMU overlay
            frame = draw_imu_overlay(frame)

            # Mode badge
            m_color = (0,200,50) if state["mode"]=="surveillance" else (0,140,255)
            cv2.rectangle(frame, (0,0), (210,26), (0,0,0), -1)
            cv2.putText(frame, f"MODE: {state['mode'].upper()}",
                        (6,18), cv2.FONT_HERSHEY_SIMPLEX, 0.52, m_color, 1)

            # Timestamp
            ts = datetime.now().strftime("%H:%M:%S")
            cv2.putText(frame, ts, (FRAME_W-72,18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160,160,160), 1)

            # Threat banner
            if threat:
                cv2.rectangle(frame, (0,FRAME_H-28), (FRAME_W,FRAME_H), (0,0,160), -1)
                cv2.putText(frame, f"THREAT: {', '.join(labels).upper()}",
                            (10, FRAME_H-8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255,255,255), 2)

            # Tipped-over banner
            if not state["upright"]:
                cv2.rectangle(frame, (0,30), (FRAME_W,62), (0,0,180), -1)
                cv2.putText(frame, "  ⚠  BOT TIPPED OVER  ⚠",
                            (10,52), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (255,255,255), 2)

            # FPS
            fps_cnt += 1
            elapsed = time.time() - fps_t
            if elapsed >= 1.0:
                state["fps"] = round(fps_cnt/elapsed, 1)
                fps_cnt, fps_t = 0, time.time()
            cv2.putText(frame, f"{state['fps']} FPS",
                        (FRAME_W-72,36), cv2.FONT_HERSHEY_SIMPLEX,
                        0.40, (120,120,120), 1)

            # Update state
            state["threat"]        = threat
            state["threat_labels"] = labels
            state["detections"]    = detections

            # Log new threats
            if threat and not state.get("_prev_threat"):
                state["log"].appendleft(
                    {"time":ts,"msg":f"THREAT: {', '.join(labels)}","type":"threat"})
            state["_prev_threat"] = threat

            # Log tip-over
            if not state["upright"] and state.get("_prev_upright", True):
                state["log"].appendleft(
                    {"time":ts,"msg":"BOT TIPPED OVER","type":"threat"})
            state["_prev_upright"] = state["upright"]

            # Encode
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with frame_lock:
                latest_b64 = base64.b64encode(buf).decode()

            sio.emit("frame", {
                "image":      latest_b64,
                "threat":     threat,
                "labels":     labels,
                "detections": detections,
                "fps":        state["fps"],
                "mode":       state["mode"],
                "upright":    state["upright"],
                "pitch":      state["pitch"],
                "roll":       state["roll"],
            })

        cap.release()


# ─── ESP32 WebSocket client ───────────────────────────────────
async def esp32_client(esp32_ip: str):
    global esp32_ws, state
    uri = f"ws://{esp32_ip}/ws"

    while True:
        try:
            log.info(f"Connecting to ESP32: {uri}")
            async with websockets.connect(uri, ping_interval=10) as ws:
                esp32_ws = ws
                state["connected_esp"] = True
                sio.emit("esp32_status", {"connected": True})
                log.info("ESP32 connected ✓")

                async for raw in ws:
                    try:
                        data = json.loads(raw)
                        t = data.get("type")

                        if t == "radar":
                            sio.emit("radar", {
                                "angle":    data["angle"],
                                "distance": data["distance"],
                                "time":     time.time()
                            })

                        elif t == "imu":
                            state["pitch"]      = data.get("pitch",      0.0)
                            state["roll"]       = data.get("roll",       0.0)
                            state["upright"]    = data.get("upright",    True)
                            state["onSlope"]    = data.get("onSlope",    False)
                            state["slopeAngle"] = data.get("slopeAngle", 0.0)
                            state["baseSpeed"]  = data.get("baseSpeed",  160)
                            state["imuFactor"]  = data.get("factor",     1.0)

                            sample = {
                                "t":      time.time(),
                                "pitch":  state["pitch"],
                                "roll":   state["roll"],
                                "speed":  state["baseSpeed"],
                            }
                            state["imu_history"].append(sample)

                            sio.emit("imu", {
                                "pitch":      state["pitch"],
                                "roll":       state["roll"],
                                "upright":    state["upright"],
                                "onSlope":    state["onSlope"],
                                "slopeAngle": state["slopeAngle"],
                                "baseSpeed":  state["baseSpeed"],
                                "factor":     state["imuFactor"],
                                "history":    list(state["imu_history"])[-20:],
                            })

                            # Forward tip-over threat to ESP32 LED
                            if not state["upright"]:
                                await ws.send(json.dumps({"threat": True}))

                        elif t == "heartbeat":
                            sio.emit("heartbeat", {
                                "mode":      data.get("mode"),
                                "threat":    data.get("threat"),
                                "rssi":      data.get("rssi"),
                                "upright":   data.get("upright", True),
                                "baseSpeed": data.get("baseSpeed", 160),
                            })

                    except (json.JSONDecodeError, KeyError):
                        pass

        except Exception as e:
            log.warning(f"ESP32 disconnected: {e} — retry in 3 s")
            esp32_ws = None
            state["connected_esp"] = False
            sio.emit("esp32_status", {"connected": False})
            await asyncio.sleep(3)


def send_esp32(data: dict):
    global esp32_ws
    if esp32_ws is None:
        return
    async def _s():
        try:
            await esp32_ws.send(json.dumps(data))
        except Exception as e:
            log.warning(f"Send failed: {e}")
    if ws_loop:
        asyncio.run_coroutine_threadsafe(_s(), ws_loop)


# ─── SocketIO events ──────────────────────────────────────────
@sio.on("drive")
def on_drive(data):
    if state["mode"] == "control":
        send_esp32({"left": data.get("left",0), "right": data.get("right",0)})

@sio.on("set_mode")
def on_set_mode(data):
    mode = data.get("mode","surveillance")
    state["mode"] = mode
    send_esp32({"mode": mode})
    ts = datetime.now().strftime("%H:%M:%S")
    state["log"].appendleft({"time":ts,"msg":f"Mode → {mode.upper()}","type":"info"})
    emit("mode_changed", {"mode": mode}, broadcast=True)

@sio.on("get_state")
def on_get_state():
    emit("full_state", {
        "mode":          state["mode"],
        "connected_esp": state["connected_esp"],
        "threat":        state["threat"],
        "fps":           state["fps"],
        "log":           list(state["log"]),
        "pitch":         state["pitch"],
        "roll":          state["roll"],
        "upright":       state["upright"],
        "onSlope":       state["onSlope"],
        "baseSpeed":     state["baseSpeed"],
        "imuFactor":     state["imuFactor"],
    })

@app.route("/status")
def status():
    return jsonify({k: state[k] for k in
        ("mode","connected_esp","threat","fps","pitch","roll","upright","baseSpeed")})


# ─── Main ─────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--esp32", default="192.168.1.101")
    parser.add_argument("--cam",   default="192.168.1.102")
    parser.add_argument("--port",  type=int, default=FLASK_PORT)
    args = parser.parse_args()

    log.info("=" * 52)
    log.info("  ASB v2.0 — Autonomous Surveillance Bot")
    log.info("=" * 52)
    log.info(f"  ESP32      : {args.esp32}")
    log.info(f"  Camera     : {args.cam}")
    log.info(f"  Dashboard  : http://localhost:{args.port}")
    log.info("=" * 52)

    model = load_model()

    # Asyncio loop for ESP32 WS
    ws_loop = asyncio.new_event_loop()
    def _run_ws():
        asyncio.set_event_loop(ws_loop)
        ws_loop.run_until_complete(esp32_client(args.esp32))
    threading.Thread(target=_run_ws, daemon=True).start()

    # Camera thread
    threading.Thread(target=camera_thread, args=(args.cam, model), daemon=True).start()

    # Flask
    sio.run(app, host="0.0.0.0", port=args.port,
            debug=False, allow_unsafe_werkzeug=True)
