/*
 * ASB — Autonomous Surveillance Bot  v2.0
 * ESP32 Main Controller  (IMU Edition)
 *
 * Components driven by this board:
 *   4× DC motors via 2× TB6612FNG
 *   1× Servo  (radar sweep)
 *   1× HC-SR04 ultrasonic
 *   1× MPU-6050 IMU  (I2C  SDA=21  SCL=22)
 *   2× LEDs
 *
 * Libraries  (Arduino Library Manager):
 *   ESPAsyncWebServer   lacamera
 *   AsyncTCP            dvarrel
 *   ESP32Servo          Kevin Harrington
 *   NewPing             Tim Eckel
 *   ArduinoJson         Benoit Blanchon
 *   MPU6050             Electronic Cats   ← NEW
 *   Wire                built-in
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>

// ─── WiFi ─────────────────────────────────────────────────────
const char* SSID     = "YOUR_WIFI_SSID";
const char* PASSWORD = "YOUR_WIFI_PASSWORD";

// ─── Motor Driver 1  — LEFT side ──────────────────────────────
#define M1_IN1  12
#define M1_IN2  13
#define M1_PWM  14
#define M2_IN1  25
#define M2_IN2  26
#define M2_PWM  27
#define STBY1   33

// ─── Motor Driver 2  — RIGHT side ─────────────────────────────
#define M3_IN1  15
#define M3_IN2   2
#define M3_PWM   4
#define M4_IN1  16
#define M4_IN2  17
#define M4_PWM   5
#define STBY2   32

// ─── Servo ────────────────────────────────────────────────────
#define SERVO_PIN   18

// ─── Ultrasonic  (ECHO moved to 35 — freed 21 for I2C) ────────
#define TRIG_PIN    19
#define ECHO_PIN    35   // ← was 21, moved to free I2C SDA
#define MAX_DIST   300

// ─── IMU  MPU-6050  I2C ───────────────────────────────────────
#define IMU_SDA  21      // default ESP32 I2C SDA
#define IMU_SCL  22      // default ESP32 I2C SCL
// AD0 → GND  → address 0x68

// ─── LEDs  (Status LED moved to 0 — freed 22 for I2C SCL) ─────
#define LED_STATUS   0   // ← was 22, moved to free I2C SCL
#define LED_ALERT   23

// ─── PWM channels ─────────────────────────────────────────────
#define PWM_FREQ        1000
#define PWM_RESOLUTION     8   // 0–255
#define CH_M1  0
#define CH_M2  1
#define CH_M3  2
#define CH_M4  3

// ─── Globals ──────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Servo          radarServo;
NewPing        sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);
MPU6050        imu;

enum BotMode { SURVEILLANCE, CONTROL };
BotMode currentMode = SURVEILLANCE;

// Servo sweep
int  servoAngle     = 0;
int  servoDir       = 1;
unsigned long lastServoMove  = 0;

// IMU state
float pitch       = 0.0f;
float roll        = 0.0f;
bool  isUpright   = true;
bool  onSlope     = false;
float slopeAngle  = 0.0f;
int   baseSpeed   = 160;   // dynamically adjusted by IMU

// Heartbeat / IMU broadcast timing
unsigned long lastIMU    = 0;
unsigned long lastStatus = 0;

bool threatDetected = false;

// ─── Motor helper ─────────────────────────────────────────────
void setMotor(int ch, int in1, int in2, int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(in1, spd > 0 ? HIGH : LOW);
  digitalWrite(in2, spd < 0 ? HIGH : LOW);
  ledcWrite(ch, abs(spd));
}

void drive(int left, int right) {
  setMotor(CH_M1, M1_IN1, M1_IN2, left);
  setMotor(CH_M2, M2_IN1, M2_IN2, left);
  setMotor(CH_M3, M3_IN1, M3_IN2, right);
  setMotor(CH_M4, M4_IN1, M4_IN2, right);
}

void stopMotors() { drive(0, 0); }

// ─── IMU update ───────────────────────────────────────────────
void updateIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw → g  (default ±2 g range → 16384 LSB/g)
  float axG = ax / 16384.0f;
  float ayG = ay / 16384.0f;
  float azG = az / 16384.0f;

  // Tilt angles in degrees
  pitch = atan2f(axG, sqrtf(ayG*ayG + azG*azG)) * 180.0f / PI;
  roll  = atan2f(ayG, sqrtf(axG*axG + azG*azG)) * 180.0f / PI;

  // Derived flags
  isUpright  = (fabsf(pitch) < 45.0f && fabsf(roll) < 45.0f);
  onSlope    = (fabsf(pitch) > 8.0f);          // >8° considered a slope
  slopeAngle = pitch;                           // positive = uphill (nose up)

  /*
   * Adaptive speed logic
   * ─────────────────────────────────────────────────────────────
   *  Uphill   (pitch > 0): increase power so the bot doesn't slow down
   *  Downhill (pitch < 0): reduce power so the bot doesn't run away
   *
   *  Formula:  factor = 1.0 + (pitch / 90) * 0.6
   *    pitch = +30°  →  factor = 1.20  →  baseSpeed = 192
   *    pitch = -20°  →  factor = 0.87  →  baseSpeed = 139
   *  Clamped to [70, 245] to protect motors and maintain control.
   */
  float factor = 1.0f + (pitch / 90.0f) * 0.6f;
  factor       = constrain(factor, 0.45f, 1.53f);
  baseSpeed    = (int)(160.0f * factor);
  baseSpeed    = constrain(baseSpeed, 70, 245);

  // Alert LED on tip-over
  if (!isUpright) {
    digitalWrite(LED_ALERT, HIGH);
  }

  // Broadcast IMU packet to all WebSocket clients
  StaticJsonDocument<192> doc;
  doc["type"]       = "imu";
  doc["pitch"]      = roundf(pitch * 10) / 10.0f;
  doc["roll"]       = roundf(roll  * 10) / 10.0f;
  doc["upright"]    = isUpright;
  doc["onSlope"]    = onSlope;
  doc["slopeAngle"] = roundf(slopeAngle * 10) / 10.0f;
  doc["baseSpeed"]  = baseSpeed;
  doc["factor"]     = roundf(factor * 100) / 100.0f;
  String msg;
  serializeJson(doc, msg);
  ws.textAll(msg);
}

// ─── Autonomous obstacle avoidance (Surveillance mode) ────────
void surveillanceAutoNav(int frontDist) {
  static unsigned long avoidUntil = 0;
  unsigned long now = millis();
  if (now < avoidUntil) return;

  if (!isUpright) {
    // Bot has tipped — do nothing until recovered
    stopMotors();
    return;
  }

  if (frontDist > 0 && frontDist < 30) {
    drive(-baseSpeed, -baseSpeed);          // back up
    delay(400);
    drive(-baseSpeed, baseSpeed);           // pivot right
    delay(350);
    stopMotors();
    avoidUntil = now + 750;
  } else {
    drive(baseSpeed, baseSpeed);            // forward patrol
  }
}

// ─── WebSocket event handler ──────────────────────────────────
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u  %s\n",
                  client->id(), client->remoteIP().toString().c_str());
    digitalWrite(LED_STATUS, HIGH);

  } else if (type == WS_EVT_DISCONNECT) {
    stopMotors();
    if (ws.count() == 0) digitalWrite(LED_STATUS, LOW);

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len) return;
    if (info->opcode != WS_TEXT) return;

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, data, len)) return;

    // Mode switch
    if (doc.containsKey("mode")) {
      String m = doc["mode"].as<String>();
      currentMode = (m == "surveillance") ? SURVEILLANCE : CONTROL;
      if (currentMode == SURVEILLANCE) stopMotors();
    }

    // Drive command  (CONTROL mode only)
    if (doc.containsKey("left") && doc.containsKey("right") && currentMode == CONTROL) {
      int l = doc["left"];
      int r = doc["right"];
      // Scale by IMU speed factor so manual drive also respects slope
      float factor = (float)baseSpeed / 160.0f;
      drive((int)(l * factor), (int)(r * factor));
    }

    // Threat flag from PC
    if (doc.containsKey("threat")) {
      threatDetected = doc["threat"].as<bool>();
      digitalWrite(LED_ALERT, threatDetected ? HIGH : LOW);
    }
  }
}

// ─── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // I2C for IMU
  Wire.begin(IMU_SDA, IMU_SCL);

  // Motor direction pins
  const int dirPins[] = {
    M1_IN1, M1_IN2, M2_IN1, M2_IN2,
    M3_IN1, M3_IN2, M4_IN1, M4_IN2,
    STBY1, STBY2
  };
  for (int p : dirPins) { pinMode(p, OUTPUT); }
  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);

  // PWM
  ledcSetup(CH_M1, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(M1_PWM, CH_M1);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(M2_PWM, CH_M2);
  ledcSetup(CH_M3, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(M3_PWM, CH_M3);
  ledcSetup(CH_M4, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(M4_PWM, CH_M4);

  // LEDs
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_ALERT,  OUTPUT);

  // Servo
  ESP32PWM::allocateTimer(0);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);
  radarServo.write(90);

  // IMU
  imu.initialize();
  if (imu.testConnection()) {
    Serial.println("[IMU] MPU-6050 OK");
    // Calibration offsets (run MPU6050_calibration sketch to get yours)
    imu.setXAccelOffset(0);
    imu.setYAccelOffset(0);
    imu.setZAccelOffset(0);
    imu.setXGyroOffset(0);
    imu.setYGyroOffset(0);
    imu.setZGyroOffset(0);
  } else {
    Serial.println("[IMU] MPU-6050 NOT FOUND — check wiring");
  }

  // WiFi
  WiFi.begin(SSID, PASSWORD);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
  }
  digitalWrite(LED_STATUS, LOW);
  Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());

  // WebSocket + HTTP
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    String ip = WiFi.localIP().toString();
    req->send(200, "text/plain",
      "ASB v2 Online\nIP: " + ip + "\nWS: ws://" + ip + "/ws");
  });
  server.begin();
  Serial.printf("[WS]   ws://%s/ws\n", WiFi.localIP().toString().c_str());
}

// ─── Loop ─────────────────────────────────────────────────────
void loop() {
  ws.cleanupClients();
  unsigned long now = millis();

  // ── IMU read every 100 ms ──────────────────────────────────
  if (now - lastIMU >= 100) {
    lastIMU = now;
    updateIMU();
    if (!isUpright && currentMode == SURVEILLANCE) stopMotors();
  }

  // ── Servo sweep + ultrasonic ping every 30 ms ──────────────
  if (now - lastServoMove >= 30) {
    lastServoMove = now;
    servoAngle += servoDir * 3;
    if (servoAngle >= 180) { servoAngle = 180; servoDir = -1; }
    if (servoAngle <=   0) { servoAngle =   0; servoDir =  1; }
    radarServo.write(servoAngle);

    int dist = sonar.ping_cm();
    if (dist == 0) dist = MAX_DIST;

    // Radar broadcast
    StaticJsonDocument<128> rdoc;
    rdoc["type"]     = "radar";
    rdoc["angle"]    = servoAngle;
    rdoc["distance"] = dist;
    String rmsg;
    serializeJson(rdoc, rmsg);
    ws.textAll(rmsg);

    // Auto-nav uses near-front reading (±10° of 90°)
    if (currentMode == SURVEILLANCE && abs(servoAngle - 90) < 10)
      surveillanceAutoNav(dist);
  }

  // ── Heartbeat every 2 s ────────────────────────────────────
  if (now - lastStatus >= 2000) {
    lastStatus = now;
    StaticJsonDocument<160> hdoc;
    hdoc["type"]      = "heartbeat";
    hdoc["mode"]      = (currentMode == SURVEILLANCE) ? "surveillance" : "control";
    hdoc["threat"]    = threatDetected;
    hdoc["rssi"]      = WiFi.RSSI();
    hdoc["upright"]   = isUpright;
    hdoc["baseSpeed"] = baseSpeed;
    String hmsg;
    serializeJson(hdoc, hmsg);
    ws.textAll(hmsg);
  }
}
