// bench_test.ino  -  Step 3: check an ESP-FLY running ESP-FC before adding mocap.
//
// Flash this on a spare ESP32 dev board (the "ground" board). Open the Serial
// Monitor at 115200 baud, line ending "Newline" (or "Both NL & CR"), and type
// commands to send stick values to the drone over ESP-NOW.
//
// PROPS OFF for every test in this sketch.
//
// Commands
//   help                  show this list
//   status                link + channel values
//   arm / disarm          AUX1 high / low (throttle must be at 1000 to arm)
//   x                     emergency stop: disarm + throttle 1000
//   t <1000-2000>         throttle  (capped at THROTTLE_CAP below)
//   r <us> / p <us> / y <us>   roll / pitch / yaw stick, 1500 = centre
//   c                     centre roll/pitch/yaw
//   silence <ms>          stop transmitting for <ms> to watch ESP-FC failsafe
//
// Channels sent (AETR order, what ESP-FC expects by default):
//   ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw, AUX1 arm, AUX2 always high (use
//   it for ANGLE mode), AUX3/AUX4 low.

#include <Arduino.h>
#include "rclink_tx.h"

// ---- settings --------------------------------------------------------------
// Leave all zeros to pair with the first ESP-FC drone that is switched on.
// The drone's MAC is printed when it pairs; paste it here to pin it.
uint8_t DRONE_MAC[1][6] = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

const int THROTTLE_CAP = 1500;           // bench safety limit (us)
const uint32_t SEND_INTERVAL_MS = 20;    // 50 Hz, same rate as the original SBUS link
const uint32_t IDLE_DISARM_MS = 60000;   // auto-disarm after 60 s with no command
// ----------------------------------------------------------------------------

rclink::MultiTx radio;

uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 2000, 1000, 1000};
bool armed = false;
uint32_t lastSendMs = 0;
uint32_t silenceUntilMs = 0;
uint32_t lastCommandMs = 0;
bool wasPaired = false;

char line[64];
size_t lineLen = 0;

void printHelp() {
  Serial.println(F("commands: help | status | arm | disarm | x | t <us> | r <us> | p <us> | y <us> | c | silence <ms>"));
}

void printStatus() {
  Serial.print(F("link: "));
  if (radio.paired(0)) {
    Serial.print(F("paired with "));
    rclink::printMac(Serial, radio.mac(0));
    Serial.print(F(" on ch "));
    Serial.print(radio.channel());
    Serial.print(F(", last heard "));
    Serial.print(radio.heardAgeMs(0));
    Serial.println(F(" ms ago"));
  } else {
    Serial.println(F("searching (power the drone on)"));
  }
  Serial.print(F("armed=")); Serial.print(armed ? F("YES") : F("no"));
  Serial.print(F("  roll=")); Serial.print(ch[0]);
  Serial.print(F(" pitch=")); Serial.print(ch[1]);
  Serial.print(F(" thr=")); Serial.print(ch[2]);
  Serial.print(F(" yaw=")); Serial.print(ch[3]);
  Serial.print(F(" aux1=")); Serial.print(ch[4]);
  Serial.print(F(" aux2=")); Serial.println(ch[5]);
}

void stopAll(const __FlashStringHelper *why) {
  armed = false;
  ch[4] = 1000;
  ch[2] = 1000;
  Serial.print(F("DISARMED: "));
  Serial.println(why);
}

int parseUs(const char *s, bool &ok) {
  char *end;
  long v = strtol(s, &end, 10);
  ok = end != s;
  return (int)v;
}

void handleLine(char *cmd) {
  lastCommandMs = millis();
  while (*cmd == ' ') cmd++;
  char *arg = strchr(cmd, ' ');
  if (arg) { *arg = 0; arg++; while (*arg == ' ') arg++; }
  bool ok = false;

  if (!strcmp(cmd, "x") || !strcmp(cmd, "stop")) {
    stopAll(F("emergency stop"));
  } else if (!strcmp(cmd, "help")) {
    printHelp();
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "arm")) {
    if (ch[2] > 1050) {
      Serial.println(F("lower the throttle first (t 1000)"));
    } else if (!radio.paired(0)) {
      Serial.println(F("not paired yet"));
    } else {
      armed = true;
      ch[4] = 2000;
      Serial.println(F("ARM switch on (ESP-FC may still refuse: check Setup tab arming flags)"));
    }
  } else if (!strcmp(cmd, "disarm")) {
    stopAll(F("disarm command"));
  } else if (!strcmp(cmd, "c")) {
    ch[0] = ch[1] = ch[3] = 1500;
    Serial.println(F("sticks centred"));
  } else if (arg && (!strcmp(cmd, "t") || !strcmp(cmd, "r") || !strcmp(cmd, "p") || !strcmp(cmd, "y"))) {
    int v = parseUs(arg, ok);
    if (!ok) { Serial.println(F("need a number, e.g. t 1100")); return; }
    v = rclink::clampInt(v, 1000, 2000);
    if (cmd[0] == 't') {
      if (v > THROTTLE_CAP) { v = THROTTLE_CAP; Serial.println(F("(throttle capped)")); }
      ch[2] = v;
    } else if (cmd[0] == 'r') ch[0] = v;
    else if (cmd[0] == 'p') ch[1] = v;
    else ch[3] = v;
    printStatus();
  } else if (arg && !strcmp(cmd, "silence")) {
    int ms = parseUs(arg, ok);
    if (!ok || ms <= 0) { Serial.println(F("e.g. silence 2000")); return; }
    silenceUntilMs = millis() + (uint32_t)ms;
    Serial.print(F("not transmitting for ")); Serial.print(ms); Serial.println(F(" ms - ESP-FC should show RX loss / failsafe"));
  } else {
    Serial.print(F("unknown command: ")); Serial.println(cmd);
    printHelp();
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineLen > 0) {
        line[lineLen] = 0;
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== ESP-FLY / ESP-FC bench test  (PROPS OFF) ==="));
  if (!radio.begin(DRONE_MAC, 1, &Serial)) {
    Serial.println(F("ESP-NOW failed to start"));
  }
  Serial.println(F("Searching for the drone... power it on now."));
  printHelp();
  lastCommandMs = millis();
}

void loop() {
  readSerial();
  radio.update();

  const uint32_t now = millis();

  if (radio.paired(0) != wasPaired) {
    wasPaired = radio.paired(0);
    if (!wasPaired && armed) stopAll(F("link lost"));
  }
  if (armed && now - lastCommandMs > IDLE_DISARM_MS) stopAll(F("no command for 60 s"));

  if ((int32_t)(now - silenceUntilMs) < 0) return;  // failsafe test: say nothing

  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    radio.send(0, ch);
  }
}
