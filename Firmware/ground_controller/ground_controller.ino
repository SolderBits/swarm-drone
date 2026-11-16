#include <Arduino.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include "rclink_tx.h"

#define NUM_DRONES 2

uint8_t DRONE_MACS[NUM_DRONES][6] = {
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

#define SERIAL_BAUD 1000000

#define MAX_VEL 100
#define Z_GAIN 0.7
#define ROTOR_RADIUS 0.0275

#define PID_SAMPLE_MS 100

#define RC_SEND_INTERVAL_MS 20
#define PING_TIMEOUT_MS 2000
#define POS_TIMEOUT_MS 500
#define LINK_TIMEOUT_MS 3000
#define ARM_THROTTLE_HOLD_MS 100

#define STATUS_PRINT_MS 0

static inline float sbusToUs(float sbus) { return 880.0f + sbus * 0.625f; }
static inline uint16_t toChannel(float us) {
  if (us < 1000.0f) us = 1000.0f;
  if (us > 2000.0f) us = 2000.0f;
  return (uint16_t)(us + 0.5f);
}

void resetPid(PID &pid, double min, double max) {

  pid.SetOutputLimits(0.0, 1.0);
  pid.SetOutputLimits(-1.0, 0.0);
  pid.SetOutputLimits(min, max);
}

struct Drone {
  bool armed = false;
  uint32_t armedAtMs = 0;
  uint32_t lastPingMs = 0;
  uint32_t lastPosMs = 0;
  uint32_t lastSendMs = 0;

  int xTrim = 0, yTrim = 0, zTrim = 0, yawTrim = 0;
  double groundEffectCoef = 28, groundEffectOffset = -0.035;

  double xPosSetpoint = 0, xPos = 0;
  double yPosSetpoint = 0, yPos = 0;
  double zPosSetpoint = 0, zPos = 0;
  double yawPosSetpoint = 0, yawPos = 0, yawPosOutput = 0;

  double xVelSetpoint = 0, xVel = 0, xVelOutput = 0;
  double yVelSetpoint = 0, yVel = 0, yVelOutput = 0;
  double zVelSetpoint = 0, zVel = 0, zVelOutput = 0;

  PID xPosPID, yPosPID, zPosPID, yawPosPID;

  PID xVelPID, yVelPID, zVelPID;

  Drone()
      : xPosPID(&xPos, &xVelSetpoint, &xPosSetpoint, 1, 0, 0, DIRECT),
        yPosPID(&yPos, &yVelSetpoint, &yPosSetpoint, 1, 0, 0, DIRECT),
        zPosPID(&zPos, &zVelSetpoint, &zPosSetpoint, 1.5, 0, 0, DIRECT),
        yawPosPID(&yawPos, &yawPosOutput, &yawPosSetpoint, 0.3, 0.1, 0.05, DIRECT),
        xVelPID(&xVel, &xVelOutput, &xVelSetpoint, 0.2, 0.03, 0.05, DIRECT),
        yVelPID(&yVel, &yVelOutput, &yVelSetpoint, 0.2, 0.03, 0.05, DIRECT),
        zVelPID(&zVel, &zVelOutput, &zVelSetpoint, 0.3, 0.1, 0.05, DIRECT) {}

  void begin() {
    PID *all[] = {&xPosPID, &yPosPID, &zPosPID, &yawPosPID, &xVelPID, &yVelPID, &zVelPID};
    for (PID *p : all) {
      p->SetMode(AUTOMATIC);
      p->SetSampleTime(PID_SAMPLE_MS);
    }
    resetAll();
  }

  void resetAll() {
    resetPid(xPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yPosPID, -MAX_VEL, MAX_VEL);
    resetPid(zPosPID, -MAX_VEL, MAX_VEL);
    resetPid(yawPosPID, -1, 1);
    resetPid(xVelPID, -1, 1);
    resetPid(yVelPID, -1, 1);
    resetPid(zVelPID, -1, 1);
  }

  void compute() {
    xPosPID.Compute();
    yPosPID.Compute();
    zPosPID.Compute();
    yawPosPID.Compute();
    xVelPID.Compute();
    yVelPID.Compute();
    zVelPID.Compute();
  }

  void channels(uint32_t now, uint16_t out[8]) {
    const float xS = 992 + (xVelOutput * 811) + xTrim;

    const float yS = 992 - (yVelOutput * 811) - yTrim;
    const float yawS = 992 + (yawPosOutput * 811) + yawTrim;
    float zS = 992 + (Z_GAIN * zVelOutput * 811) + zTrim;

    const double h = zPos - groundEffectOffset;
    double groundEffect = 0.0;
    if (h > 0.001) groundEffect = 1 - groundEffectCoef * pow((2 * ROTOR_RADIUS) / (4 * h), 2);
    if (groundEffect < 0) groundEffect = 0;
    zS *= groundEffect;

    const bool throttleLive = armed && (now - armedAtMs > ARM_THROTTLE_HOLD_MS);
    if (!throttleLive) zS = 172;

    out[0] = toChannel(sbusToUs(yS));
    out[1] = toChannel(sbusToUs(xS));
    out[2] = toChannel(sbusToUs(zS));
    out[3] = toChannel(sbusToUs(yawS));
    out[4] = armed ? 2000 : 1000;
    out[5] = 2000;
    out[6] = 1000;
    out[7] = 1000;
  }
};

Drone drones[NUM_DRONES];
rclink::MultiTx radio;

bool linkOk(int idx) { return radio.paired(idx) && radio.heardAgeMs(idx) <= LINK_TIMEOUT_MS; }

#if ARDUINOJSON_VERSION_MAJOR >= 7
JsonDocument json;
#else
StaticJsonDocument<1024> json;
#endif

void disarm(int idx, const char *why) {
  Drone &d = drones[idx];
  if (!d.armed) return;
  d.armed = false;
  d.resetAll();
  Serial.print("[ground] drone ");
  Serial.print(idx);
  Serial.print(" DISARMED: ");
  Serial.print(why);
  Serial.println("  (toggle arm off/on in the UI to re-arm)");
}

void handleMessage(int idx, const char *text, size_t len) {
  if (idx < 0 || idx >= NUM_DRONES) return;
  DeserializationError err = deserializeJson(json, text, len);
  if (err) {
    Serial.print("[ground] bad json: ");
    Serial.println(err.c_str());
    return;
  }
  Drone &d = drones[idx];
  const uint32_t now = millis();

  if (json["pos"].is<JsonArray>() && json["vel"].is<JsonArray>()) {
    d.xPos = json["pos"][0].as<double>();
    d.yPos = json["pos"][1].as<double>();
    d.zPos = json["pos"][2].as<double>();
    d.yawPos = json["pos"][3].as<double>();
    d.xVel = json["vel"][0].as<double>();
    d.yVel = json["vel"][1].as<double>();
    d.zVel = json["vel"][2].as<double>();
    d.lastPosMs = now;
  } else if (json["armed"].is<bool>()) {
    const bool want = json["armed"].as<bool>();
    if (want && !d.armed) {
      if (!linkOk(idx)) {
        Serial.print("[ground] drone ");
        Serial.print(idx);
        Serial.println(" has no ESP-NOW link, ignoring arm");
      } else {
        d.armed = true;
        d.armedAtMs = now;
        d.resetAll();
      }
    } else if (!want) {
      disarm(idx, "UI");
    }
  } else if (json["setpoint"].is<JsonArray>()) {
    d.xPosSetpoint = json["setpoint"][0].as<double>();
    d.yPosSetpoint = json["setpoint"][1].as<double>();
    d.zPosSetpoint = json["setpoint"][2].as<double>();
  } else if (json["pid"].is<JsonArray>()) {
    JsonArray p = json["pid"].as<JsonArray>();
    if (p.size() >= 17) {
      double k[17];
      for (int i = 0; i < 17; i++) k[i] = p[i].as<double>();
      d.xPosPID.SetTunings(k[0], k[1], k[2]);
      d.yPosPID.SetTunings(k[0], k[1], k[2]);
      d.zPosPID.SetTunings(k[3], k[4], k[5]);
      d.yawPosPID.SetTunings(k[6], k[7], k[8]);
      d.xVelPID.SetTunings(k[9], k[10], k[11]);
      d.yVelPID.SetTunings(k[9], k[10], k[11]);
      d.zVelPID.SetTunings(k[12], k[13], k[14]);
      d.groundEffectCoef = k[15];
      d.groundEffectOffset = k[16];
    }
  } else if (json["trim"].is<JsonArray>()) {
    d.xTrim = json["trim"][0].as<int>();
    d.yTrim = json["trim"][1].as<int>();
    d.zTrim = json["trim"][2].as<int>();
    d.yawTrim = json["trim"][3].as<int>();
  }

  d.lastPingMs = now;
}

void readSerial() {
  static char buf[1024];
  static size_t len = 0;
  static int idx = -1, depth = 0;
  static bool inString = false, escaped = false;

  while (Serial.available()) {
    const char c = (char)Serial.read();

    if (idx < 0 || len == 0) {
      if (idx >= 0 && c == '{') {

      } else {
        idx = (c >= '0' && c <= '9') ? c - '0' : -1;
        len = 0; depth = 0; inString = false; escaped = false;
        continue;
      }
    }

    if (len >= sizeof(buf) - 1) {
      idx = -1; len = 0;
      continue;
    }
    buf[len++] = c;

    if (inString) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') inString = false;
    } else if (c == '"') {
      inString = true;
    } else if (c == '{') {
      depth++;
    } else if (c == '}') {
      if (--depth == 0) {
        buf[len] = 0;
        handleMessage(idx, buf, len);
        idx = -1; len = 0;
      }
    }
  }
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println();
  Serial.println("[ground] Low-Cost-Mocap ground controller for ESP-FC / ESP-FLY");

  for (int i = 0; i < NUM_DRONES; i++) drones[i].begin();

  if (NUM_DRONES > 1) {
    for (int i = 0; i < NUM_DRONES; i++) {
      if (rclink::macIsZero(DRONE_MACS[i])) {
        Serial.println("[ground] WARNING: DRONE_MACS not set - drone numbers follow power-on order. Pin the MACs for 2+ drones.");
        break;
      }
    }
  }

  if (!radio.begin(DRONE_MACS, NUM_DRONES, &Serial)) {
    Serial.println("[ground] ESP-NOW failed to start");
  }
  Serial.println("[ground] scanning for drones - power them on now");
}

void loop() {
  readSerial();
  radio.update();

  const uint32_t now = millis();

  for (int i = 0; i < NUM_DRONES; i++) {
    Drone &d = drones[i];

    if (d.armed) {
      const uint32_t posRef = d.lastPosMs > d.armedAtMs ? d.lastPosMs : d.armedAtMs;
      if (now - d.lastPingMs > PING_TIMEOUT_MS) disarm(i, "no messages from the PC");
      else if (now - posRef > POS_TIMEOUT_MS) disarm(i, "mocap position stopped arriving");
      else if (!linkOk(i)) disarm(i, "drone stopped answering over ESP-NOW");
    }

    if (!d.armed) d.resetAll();
    d.compute();

    if (now - d.lastSendMs >= RC_SEND_INTERVAL_MS) {
      d.lastSendMs = now;
      uint16_t ch[8];
      d.channels(now, ch);
      radio.send(i, ch);
    }
  }

#if STATUS_PRINT_MS > 0
  static uint32_t lastStatus = 0;
  if (now - lastStatus >= STATUS_PRINT_MS) {
    lastStatus = now;
    for (int i = 0; i < NUM_DRONES; i++) {
      Serial.print("[status] drone ");
      Serial.print(i);
      Serial.print(radio.paired(i) ? " paired" : " searching");
      Serial.print(drones[i].armed ? " ARMED" : " disarmed");
      Serial.print(" pos age ");
      Serial.print(now - drones[i].lastPosMs);
      Serial.println(" ms");
    }
  }
#endif
}
