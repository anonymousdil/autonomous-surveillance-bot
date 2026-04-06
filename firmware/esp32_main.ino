/*
 * ASB — Autonomous Surveillance Bot
 * ESP32 Main Controller
 * 
 * Handles:
 *  - 4WD Motor control via 2x TB6612FNG
 *  - Servo sweep for ultrasonic radar
 *  - WebSocket server for PC communication
 *  - LED status indicators
 *  - Two modes: SURVEILLANCE and CONTROL
 * 
 * Libraries needed (install via Arduino Library Manager):
 *  - ESPAsyncWebServer  (by lacamera)
 *  - AsyncTCP           (by dvarrel)
 *  - ESP32Servo         (by Kevin Harrington)
 *  - NewPing            (by Tim Eckel)
 *  - ArduinoJson        (by Benoit Blanchon)
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include <ArduinoJson.h>

// ─── WiFi Credentials ─────────────────────────────────────────
const char* SSID     = "YOUR_WIFI_SSID";
const char* PASSWORD = "YOUR_WIFI_PASSWORD";

// ─── Motor Driver 1 — Left Side (TB6612FNG #1) ────────────────
#define MOTOR1_IN1  12   // Left Front — forward
#define MOTOR1_IN2  13   // Left Front — backward
#define MOTOR1_PWM  14   // Left Front — speed
#define MOTOR2_IN1  25   // Left Rear  — forward
#define MOTOR2_IN2  26   // Left Rear  — backward
#define MOTOR2_PWM  27   // Left Rear  — speed

// ─── Motor Driver 2 — Right Side (TB6612FNG #2) ───────────────
#define MOTOR3_IN1  15   // Right Front — forward
#define MOTOR3_IN2   2   // Right Front — backward
#define MOTOR3_PWM   4   // Right Front — speed
#define MOTOR4_IN1  16   // Right Rear  — forward
#define MOTOR4_IN2  17   // Right Rear  — backward
#define MOTOR4_PWM   5   // Right Rear  — speed

// STBY pins — set HIGH to enable motor drivers
#define STBY1       33
#define STBY2       32

// ─── Servo & Ultrasonic ───────────────────────────────────────
#define SERVO_PIN   18
#define TRIG_PIN    19
#define ECHO_PIN    21
#define MAX_DIST    300   // cm

// ─── LEDs ─────────────────────────────────────────────────────
#define LED_STATUS  22   // Green: connected
#define LED_ALERT   23   // Red:   threat detected

// ─── PWM Channels ─────────────────────────────────────────────
#define PWM_FREQ      1000
#define PWM_RESOLUTION   8   // 0–255
#define CH_M1  0
#define CH_M2  1
#define CH_M3  2
#define CH_M4  3

// ─── Globals ──────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Servo radarServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

enum BotMode { SURVEILLANCE, CONTROL };
BotMode currentMode = SURVEILLANCE;

int servoAngle       = 0;
int servoDirection   = 1;   // 1 = increasing, -1 = decreasing
unsigned long lastServoMove  = 0;
unsigned long lastPing       = 0;
unsigned long lastStatusSend = 0;

bool threatDetected  = false;

// ─── Motor Helpers ────────────────────────────────────────────
void setMotor(int pwmCh, int in1, int in2, int speed) {
  // speed: -255 to +255
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledcWrite(pwmCh, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(CH_M1, MOTOR1_IN1, MOTOR1_IN2, leftSpeed);
  setMotor(CH_M2, MOTOR2_IN1, MOTOR2_IN2, leftSpeed);
  setMotor(CH_M3, MOTOR3_IN1, MOTOR3_IN2, rightSpeed);
  setMotor(CH_M4, MOTOR4_IN1, MOTOR4_IN2, rightSpeed);
}

void stopMotors() {
  drive(0, 0);
}

// ─── WebSocket Event Handler ──────────────────────────────────
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
    digitalWrite(LED_STATUS, HIGH);

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected\n", client->id());
    stopMotors();
    if (ws.count() == 0) digitalWrite(LED_STATUS, LOW);

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {

      String msg = String((char*)data).substring(0, len);
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, msg);
      if (err) return;

      // ── Mode switch ──
      if (doc.containsKey("mode")) {
        String m = doc["mode"].as<String>();
        currentMode = (m == "surveillance") ? SURVEILLANCE : CONTROL;
        Serial.printf("[MODE] Switched to %s\n", m.c_str());
        if (currentMode == SURVEILLANCE) stopMotors();
      }

      // ── Drive command (CONTROL mode only) ──
      if (doc.containsKey("left") && doc.containsKey("right")) {
        if (currentMode == CONTROL) {
          int l = doc["left"];
          int r = doc["right"];
          drive(l, r);
        }
      }

      // ── Threat alert from PC (OpenCV detection) ──
      if (doc.containsKey("threat")) {
        threatDetected = doc["threat"].as<bool>();
        digitalWrite(LED_ALERT, threatDetected ? HIGH : LOW);
      }

      // ── LED direct control ──
      if (doc.containsKey("led_status")) {
        digitalWrite(LED_STATUS, doc["led_status"].as<bool>() ? HIGH : LOW);
      }
    }
  }
}

// ─── Autonomous Obstacle Avoidance (Surveillance Mode) ────────
void surveillanceAutoNav(int frontDist) {
  static unsigned long avoidUntil = 0;
  unsigned long now = millis();

  if (now < avoidUntil) return;  // currently executing avoidance maneuver

  if (frontDist > 0 && frontDist < 30) {
    // Obstacle within 30 cm — back up and turn
    drive(-150, -150);
    delay(400);
    drive(-150, 150);   // pivot right
    delay(350);
    stopMotors();
    avoidUntil = now + 750;
  } else {
    drive(160, 160);    // gentle forward patrol
  }
}

// ─── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motor pins
  int inPins[] = {MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2,
                  MOTOR3_IN1, MOTOR3_IN2, MOTOR4_IN1, MOTOR4_IN2,
                  STBY1, STBY2};
  for (int p : inPins) { pinMode(p, OUTPUT); }
  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);

  // PWM channels
  ledcSetup(CH_M1, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(MOTOR1_PWM, CH_M1);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(MOTOR2_PWM, CH_M2);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(MOTOR3_PWM, CH_M3);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(MOTOR4_PWM, CH_M4);

  // LEDs
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_ALERT,  OUTPUT);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_ALERT,  LOW);

  // Servo
  ESP32PWM::allocateTimer(0);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);
  radarServo.write(90);

  // WiFi
  WiFi.begin(SSID, PASSWORD);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));  // blink while connecting
  }
  Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  digitalWrite(LED_STATUS, LOW);

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Simple HTTP status page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    String ip = WiFi.localIP().toString();
    req->send(200, "text/plain",
      "ASB Online\nIP: " + ip + "\nWebSocket: ws://" + ip + "/ws");
  });

  server.begin();
  Serial.printf("[SERVER] WebSocket at ws://%s/ws\n",
                WiFi.localIP().toString().c_str());
}

// ─── Loop ─────────────────────────────────────────────────────
void loop() {
  ws.cleanupClients();
  unsigned long now = millis();

  // ── Servo sweep + ultrasonic ping every 30ms ──
  if (now - lastServoMove >= 30) {
    lastServoMove = now;
    servoAngle += (servoDirection * 3);  // 3° per step

    if (servoAngle >= 180) { servoAngle = 180; servoDirection = -1; }
    if (servoAngle <= 0)   { servoAngle = 0;   servoDirection =  1; }

    radarServo.write(servoAngle);

    // Ping and broadcast radar data
    int dist = sonar.ping_cm();
    if (dist == 0) dist = MAX_DIST;  // no echo = max range

    StaticJsonDocument<128> radarDoc;
    radarDoc["type"]     = "radar";
    radarDoc["angle"]    = servoAngle;
    radarDoc["distance"] = dist;
    radarDoc["mode"]     = (currentMode == SURVEILLANCE) ? "surveillance" : "control";

    String radarMsg;
    serializeJson(radarDoc, radarMsg);
    ws.textAll(radarMsg);

    // Auto-nav uses front-facing measurement (near 90°)
    if (currentMode == SURVEILLANCE && abs(servoAngle - 90) < 10) {
      surveillanceAutoNav(dist);
    }
  }

  // ── Send heartbeat every 2s ──
  if (now - lastStatusSend >= 2000) {
    lastStatusSend = now;
    StaticJsonDocument<128> hbDoc;
    hbDoc["type"]    = "heartbeat";
    hbDoc["mode"]    = (currentMode == SURVEILLANCE) ? "surveillance" : "control";
    hbDoc["threat"]  = threatDetected;
    hbDoc["rssi"]    = WiFi.RSSI();
    String hbMsg;
    serializeJson(hbDoc, hbMsg);
    ws.textAll(hbMsg);
  }
}
