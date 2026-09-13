/*
=====================================================================
 XLR8 COMPETITION BOT - FINAL FIRMWARE
 Raspberry Pi Pico W
=====================================================================

 MODES  (one button TOGGLES between the two - no cycling, no IR,
 no line-follower: just these two)
 --------------------------------------------------------------------
  0 = GESTURE   (boots here - primary competition driving mode)
      ESP-01 -> WiFi UDP -> Pico W -> L298N. Pure manual control -
      no ultrasonic, no buzzer, no throttle capping of any kind.
      What you tilt is exactly what you get, every time, including
      full power on inclines/ramps with nothing second-guessing you.

  1 = AUTO      (bonus / "out-of-the-box" extra, not required or
                 scored by the rules, off by default)
      Fully autonomous: cruises forward pinging ahead, and when
      something's close it stops, sweeps the servo-mounted sensor
      LEFT -> CENTER -> RIGHT, and turns toward whichever side has
      more room. If both sides are tight it takes a longer escape
      turn instead of a normal one. The ultrasonic sensor and servo
      are ONLY EVER active in this mode.

 BUTTON
 --------------------------------------------------------------------
 One button toggles:  GESTURE <-> AUTO

 SAFETY
 --------------------------------------------------------------------
 - Motor outputs start at zero.
 - Every mode change immediately stops the motors.
 - GESTURE requires a NEW UDP packet after any mode switch before it
   will drive again, so a stale tilt reading can never move the bot.
 - UDP timeout stops the motors.
 - Malformed / NaN / Inf packets are rejected outright.
 - Motor output is slew-rate limited (accel/decel are dt-scaled, so
   the ramp is real seconds, not "per loop tick").

 IMPORTANT HARDWARE
 --------------------------------------------------------------------
 L298N:
   Left  IN1 = GPIO 6   Left  IN2 = GPIO 7   Left  ENA = GPIO 5
   Right IN3 = GPIO 3   Right IN4 = GPIO 4   Right ENB = GPIO 2

 Servo (ultrasonic mount):
   GPIO 22

 HC-SR04:
   TRIG = GPIO 14   ECHO = GPIO 15
   !!! ECHO MUST GO THROUGH A VOLTAGE DIVIDER !!! Pico GPIO is
   3.3V-only; the HC-SR04 echo pin drives 5V and WILL damage the
   pin if wired directly (e.g. 1k series + 2k to GND).

 Button:
   GPIO 16 -> button -> GND, internal pull-up enabled

 WiFi:
   Pico W creates a soft-AP. The ESP-01 transmitter connects to it
   and sends ASCII lines:   "x, y, z\n"

 Machine-spec reminder (from the XLR8 rules): keep the voltage
 between any two points on the bot <= 12V at all times, onboard
 battery only, whole machine (incl. battery) inside 25x25x25cm at
 all times during the run.

 NOTE: with the ultrasonic sensor confined to AUTO mode (an unscored
 extra), the "Ultrasonic Obstacle Detection Radar Module" bonus from
 the rules - which is scored as a driver-assist during manual
 driving - is not being pursued in this build. Clean, uninterrupted
 manual control was prioritized over that bonus.
=====================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// =====================================================================
// NETWORK
// =====================================================================

static const char* AP_SSID = "Hermes0057";
static const char* AP_PASS = "ehan0057";

static const uint16_t UDP_PORT = 80;
static const int AP_CHANNEL = 6;

#define SEND_ACK 1   // reply "A" to every packet so the ESP-01 stays associated


// =====================================================================
// PIN CONFIGURATION
// =====================================================================

static const uint8_t LEFT_IN1 = 6,  LEFT_IN2 = 7,  LEFT_EN = 5;
static const uint8_t RIGHT_IN1 = 3, RIGHT_IN2 = 4, RIGHT_EN = 2;

static const uint8_t TRIG_PIN = 14, ECHO_PIN = 15;   // ECHO via divider!
static const uint8_t SERVO_PIN = 22;
static const uint8_t BUTTON_PIN = 16;


// =====================================================================
// MOTOR CONFIGURATION
// =====================================================================

static const uint32_t PWM_FREQ_HZ = 1000;

static const bool SWAP_MOTORS  = false;
static const bool INVERT_LEFT  = false;
static const bool INVERT_RIGHT = false;

static const float MIN_DUTY   = 0.25f;  // smallest duty that still turns the wheels
static const float MIN_OUTPUT = 0.04f;  // below this, treat as stopped


// =====================================================================
// GESTURE (TILT) CONTROL
// =====================================================================

static const float TILT_RANGE = 10.0f;  // expected +/- tilt range per axis

static const float PITCH_DEADZONE = 0.08f;
static const float ROLL_DEADZONE  = 0.08f;

static const float THROTTLE_EXP = 1.6f;  // >1 = gentle near center, sharper at full tilt
static const float STEER_EXP    = 1.6f;

static const float STEER_GAIN = 0.75f;
static const float STEERING_SPEED_FACTOR = 0.35f;  // steering authority shrinks at speed

static const float YAW_DAMPING   = 0.25f;  // set to 0 if z isn't a gyro rate
static const float GYRO_DEADZONE = 0.03f;

static const float MPU_FILTER_ALPHA = 0.35f;  // EMA on incoming tilt


// =====================================================================
// MOTOR MOTION SHAPING  (dt-scaled -> frame-rate independent)
// =====================================================================

static const float ACCEL_LIMIT = 3.2f;   // 0..1 output units per second
static const float DECEL_LIMIT = 4.8f;


// =====================================================================
// IMU CALIBRATION
// =====================================================================

static const uint32_t CALIBRATION_MS = 2000;   // hold bot level & still at boot


// =====================================================================
// UDP / LINK SAFETY
// =====================================================================

static const uint32_t LINK_TIMEOUT_MS = 400;


// =====================================================================
// ULTRASONIC - AUTO MODE ONLY (idle in GESTURE)
// =====================================================================

static const uint32_t RADAR_PING_MS   = 65;
static const uint32_t ECHO_TIMEOUT_US = 12000UL;  // ~2 m useful range
static const float    US_MAX_CM       = 400.0f;

static const float AUTO_FORWARD_SPEED = 0.42f;
static const float AUTO_TURN_SPEED    = 0.58f;
static const float OBSTACLE_NEAR_CM   = 22.0f;

static const int AUTO_LEFT_ANGLE   = 45;
static const int AUTO_CENTER_ANGLE = 90;
static const int AUTO_RIGHT_ANGLE  = 135;

static const uint32_t SWEEP_SETTLE_MS    = 160;  // servo settle before each ping
static const uint32_t AUTO_TURN_MS       = 420;  // normal turn duration
static const uint32_t AUTO_ESCAPE_TURN_MS = 650; // longer turn when both sides are tight

static const uint32_t BUTTON_DEBOUNCE_MS = 45;


// =====================================================================
// MODES
// =====================================================================

enum Mode { MODE_GESTURE, MODE_AUTO, MODE_COUNT };
static Mode currentMode = MODE_GESTURE;

enum AutoState { AUTO_FORWARD, AUTO_SCAN_LEFT, AUTO_SCAN_CENTER, AUTO_SCAN_RIGHT, AUTO_TURN };
static AutoState autoState = AUTO_FORWARD;


// =====================================================================
// OBJECTS
// =====================================================================

WiFiUDP udp;
Servo   radarServo;


// =====================================================================
// STATE
// =====================================================================

static float gx = 0, gy = 0, gz = 0;
static float filteredGx = 0, filteredGy = 0, filteredGz = 0;
static float pitchOffset = 0, rollOffset = 0, yawOffset = 0;
static bool  calibrated = false;

static uint32_t lastDataMs = 0;
static bool     freshPacketSinceModeSwitch = false;

static float leftOut = 0, rightOut = 0;
static bool  motorsStopped = true;

static int      lastButtonReading = HIGH, buttonState = HIGH;
static uint32_t lastDebounceMs = 0;

static uint32_t lastRadarPingMs = 0;
static float    lastRadarCm = US_MAX_CM;

static float    autoDistL = US_MAX_CM, autoDistC = US_MAX_CM, autoDistR = US_MAX_CM;
static uint32_t autoStateStart = 0;

static uint32_t lastLoopMs = 0;
static uint32_t lastBlinkMs = 0;
static bool     ledOn = false;


// =====================================================================
// HELPERS
// =====================================================================

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float signedPow(float n, float e) {
  return (n < 0.0f ? -1.0f : 1.0f) * powf(fabsf(n), e);
}

static inline float applyDeadzone(float v, float dz) {
  if (fabsf(v) < dz) return 0.0f;
  float sign = (v >= 0.0f) ? 1.0f : -1.0f;
  return sign * (fabsf(v) - dz) / (1.0f - dz);
}

static float emaFilter(float previous, float raw, float alpha) {
  return alpha * raw + (1.0f - alpha) * previous;
}

// dt-scaled slew: moves at most (limit * dt) per call, so the ramp
// rate is real seconds, not "per loop iteration" (loop time isn't
// constant - a blocking ultrasonic ping stretches some iterations).
static float slewToward(float current, float target, float dt) {
  float limit = (fabsf(target) > fabsf(current)) ? ACCEL_LIMIT : DECEL_LIMIT;
  if ((current > 0 && target < 0) || (current < 0 && target > 0)) limit = DECEL_LIMIT;
  float maxDelta = limit * dt;
  if (current < target) { current += maxDelta; if (current > target) current = target; }
  else if (current > target) { current -= maxDelta; if (current < target) current = target; }
  return current;
}


// =====================================================================
// LOW-LEVEL MOTOR DRIVER
// =====================================================================

static void driveOne(uint8_t in1, uint8_t in2, uint8_t en, float value, bool invert) {
  if (invert) value = -value;
  value = clampf(value, -1.0f, 1.0f);

  if (fabsf(value) < MIN_OUTPUT) {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(en, 0);
    return;
  }
  if (value > 0.0f) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }

  float duty = MIN_DUTY + (1.0f - MIN_DUTY) * fabsf(value);
  duty = clampf(duty, 0.0f, 1.0f);
  analogWrite(en, (int)(duty * 255.0f));
}

static void applyOutputs(float left, float right) {
  left  = clampf(left, -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);
  if (SWAP_MOTORS) { float t = left; left = right; right = t; }
  driveOne(LEFT_IN1, LEFT_IN2, LEFT_EN, left, INVERT_LEFT);
  driveOne(RIGHT_IN1, RIGHT_IN2, RIGHT_EN, right, INVERT_RIGHT);
}

static void stopMotors() {
  leftOut = 0.0f; rightOut = 0.0f;
  digitalWrite(LEFT_IN1, LOW);  digitalWrite(LEFT_IN2, LOW);  analogWrite(LEFT_EN, 0);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW); analogWrite(RIGHT_EN, 0);
  motorsStopped = true;
}


// =====================================================================
// ULTRASONIC
// =====================================================================

// Blocking up to ECHO_TIMEOUT_US. Returns US_MAX_CM on timeout (clear).
static float pingCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return US_MAX_CM;

  float distance = (duration * 0.0343f) / 2.0f;
  if (distance < 2.0f) return 2.0f;
  if (distance > US_MAX_CM) return US_MAX_CM;
  return distance;
}


// =====================================================================
// UDP PARSING  ("x, y, z" ASCII - matches the ESP-01 transmitter)
// =====================================================================

static bool tryParseAndStoreLine(char* raw) {
  for (char* p = raw; *p; p++) if (*p == ',' || *p == '\t') *p = ' ';

  float values[3]; int count = 0; char* save = nullptr;
  for (char* piece = strtok_r(raw, " \r\n", &save); piece; piece = strtok_r(nullptr, " \r\n", &save)) {
    if (count >= 3) return false;
    char* end = nullptr;
    float v = strtof(piece, &end);
    if (end == piece || *end != '\0') return false;
    if (!isfinite(v)) return false;   // reject NaN / Inf
    values[count++] = v;
  }
  if (count != 3) return false;

  gx = values[0]; gy = values[1]; gz = values[2];
  return true;
}

// Drain every queued datagram, ACK the sender, act only on the newest
// valid line (never replays a stale tilt queued during a slow pass).
static void drainPackets() {
  char newest[48]; bool haveNewest = false;
  IPAddress sender; uint16_t senderPort = 0; bool haveSender = false;

  int packetSize;
  while ((packetSize = udp.parsePacket()) > 0) {
    char buffer[128];
    int received = udp.read(buffer, sizeof(buffer) - 1);
    if (received < 0) received = 0;
    buffer[received] = '\0';
    sender = udp.remoteIP(); senderPort = udp.remotePort(); haveSender = true;

    for (char* p = buffer; *p; p++) if (*p == '\r') *p = '\n';
    char* save = nullptr;
    for (char* line = strtok_r(buffer, "\n", &save); line; line = strtok_r(nullptr, "\n", &save)) {
      if (*line == '\0') continue;
      strncpy(newest, line, sizeof(newest) - 1);
      newest[sizeof(newest) - 1] = '\0';
      haveNewest = true;
    }
  }

#if SEND_ACK
  if (haveSender) {
    udp.beginPacket(sender, senderPort);
    udp.write((const uint8_t*)"A", 1);
    udp.endPacket();
  }
#endif

  if (haveNewest) {
    char copy[48]; strncpy(copy, newest, sizeof(copy)); copy[sizeof(copy) - 1] = '\0';
    if (tryParseAndStoreLine(copy)) {
      uint32_t now = millis();
      lastDataMs = now;
      freshPacketSinceModeSwitch = true;
      filteredGx = emaFilter(filteredGx, gx, MPU_FILTER_ALPHA);
      filteredGy = emaFilter(filteredGy, gy, MPU_FILTER_ALPHA);
      filteredGz = emaFilter(filteredGz, gz, MPU_FILTER_ALPHA);
    }
  }
}


// =====================================================================
// CALIBRATION (hold the bot level & still at boot)
// =====================================================================

static void runCalibration() {
  float sumPitch = 0, sumRoll = 0, sumYaw = 0;
  uint16_t packetCount = 0;
  uint32_t start = millis();

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F(" IMU CALIBRATION"));
  Serial.println(F(" Keep robot LEVEL and STILL"));
  Serial.println(F("================================"));

  while (millis() - start < CALIBRATION_MS) {
    int packetSize;
    while ((packetSize = udp.parsePacket()) > 0) {
      char buffer[64];
      int received = udp.read(buffer, sizeof(buffer) - 1);
      if (received < 0) received = 0;
      buffer[received] = '\0';
      if (tryParseAndStoreLine(buffer)) { sumPitch += gx; sumRoll += gy; sumYaw += gz; packetCount++; }
    }
    digitalWrite(LED_BUILTIN, ((millis() / 100) & 1) ? HIGH : LOW);
  }
  digitalWrite(LED_BUILTIN, LOW);

  if (packetCount >= 5) {
    pitchOffset = sumPitch / packetCount;
    rollOffset  = sumRoll / packetCount;
    yawOffset   = sumYaw / packetCount;
    filteredGx = pitchOffset; filteredGy = rollOffset; filteredGz = yawOffset;
    calibrated = true;
    Serial.printf("[CAL] packets=%u\n", packetCount);
    Serial.printf("[CAL] pitch offset = %.3f\n", pitchOffset);
    Serial.printf("[CAL] roll offset  = %.3f\n", rollOffset);
    Serial.printf("[CAL] yaw offset   = %.3f\n", yawOffset);
  } else {
    Serial.println(F("[CAL] WARNING: insufficient packets, using zero offsets"));
    pitchOffset = 0; rollOffset = 0; yawOffset = 0;
  }
}


// =====================================================================
// MODE 0: GESTURE  (primary - pure manual, no ultrasonic involvement)
// =====================================================================

static void runGesture(uint32_t now, float dt) {
  bool linkOK = freshPacketSinceModeSwitch && ((now - lastDataMs) <= LINK_TIMEOUT_MS);
  if (!linkOK) {
    leftOut = slewToward(leftOut, 0.0f, dt);
    rightOut = slewToward(rightOut, 0.0f, dt);
    applyOutputs(leftOut, rightOut);
    if (fabsf(leftOut) < 0.01f && fabsf(rightOut) < 0.01f) motorsStopped = true;
    return;
  }
  motorsStopped = false;

  float pitch = filteredGx - pitchOffset;
  float roll  = filteredGy - rollOffset;
  float yaw   = filteredGz - yawOffset;

  float throttle = applyDeadzone(clampf(pitch / TILT_RANGE, -1.0f, 1.0f), PITCH_DEADZONE);
  throttle = signedPow(clampf(throttle, -1.0f, 1.0f), THROTTLE_EXP);

  float steering = applyDeadzone(clampf(roll / TILT_RANGE, -1.0f, 1.0f), ROLL_DEADZONE);
  steering = signedPow(clampf(steering, -1.0f, 1.0f), STEER_EXP) * STEER_GAIN;

  float yawRate = applyDeadzone(yaw, GYRO_DEADZONE);
  steering = clampf(steering - yawRate * YAW_DAMPING, -1.0f, 1.0f);

  float speed = fabsf((leftOut + rightOut) * 0.5f);
  float steerGain = clampf(1.0f - STEERING_SPEED_FACTOR * speed, 0.35f, 1.0f);
  steering *= steerGain;

  float targetLeft  = clampf(throttle + steering, -1.0f, 1.0f);
  float targetRight = clampf(throttle - steering, -1.0f, 1.0f);

  leftOut  = slewToward(leftOut,  targetLeft,  dt);
  rightOut = slewToward(rightOut, targetRight, dt);
  applyOutputs(leftOut, rightOut);
}


// =====================================================================
// MODE 1: AUTO  (bonus / extra credit only - not required or scored)
// =====================================================================

static void beginAutoScan() {
  autoDistL = US_MAX_CM; autoDistC = US_MAX_CM; autoDistR = US_MAX_CM;
  radarServo.write(AUTO_LEFT_ANGLE);
  autoState = AUTO_SCAN_LEFT;
  autoStateStart = millis();
}

static void runAuto(uint32_t now, float dt) {
  switch (autoState) {

    case AUTO_FORWARD: {
      radarServo.write(AUTO_CENTER_ANGLE);
      if (now - lastRadarPingMs >= RADAR_PING_MS) { lastRadarPingMs = now; lastRadarCm = pingCm(); }

      if (lastRadarCm <= OBSTACLE_NEAR_CM) {
        leftOut = slewToward(leftOut, 0.0f, dt);
        rightOut = slewToward(rightOut, 0.0f, dt);
        applyOutputs(leftOut, rightOut);
        if (fabsf(leftOut) < 0.03f && fabsf(rightOut) < 0.03f) beginAutoScan();
        break;
      }
      leftOut  = slewToward(leftOut,  AUTO_FORWARD_SPEED, dt);
      rightOut = slewToward(rightOut, AUTO_FORWARD_SPEED, dt);
      applyOutputs(leftOut, rightOut);
      break;
    }

    case AUTO_SCAN_LEFT: {
      if (now - autoStateStart < SWEEP_SETTLE_MS) break;
      autoDistL = pingCm();
      Serial.printf("[AUTO] LEFT   %.1f cm\n", autoDistL);
      radarServo.write(AUTO_CENTER_ANGLE);
      autoState = AUTO_SCAN_CENTER;
      autoStateStart = now;
      break;
    }

    case AUTO_SCAN_CENTER: {
      if (now - autoStateStart < SWEEP_SETTLE_MS) break;
      autoDistC = pingCm();
      Serial.printf("[AUTO] CENTER %.1f cm\n", autoDistC);
      radarServo.write(AUTO_RIGHT_ANGLE);
      autoState = AUTO_SCAN_RIGHT;
      autoStateStart = now;
      break;
    }

    case AUTO_SCAN_RIGHT: {
      if (now - autoStateStart < SWEEP_SETTLE_MS) break;
      autoDistR = pingCm();
      Serial.printf("[AUTO] RIGHT  %.1f cm\n", autoDistR);
      radarServo.write(AUTO_CENTER_ANGLE);

      // Steer toward whichever side had more clearance. requiredTurnTime
      // inside AUTO_TURN re-reads autoDistL/autoDistR every frame and
      // picks the escape duration on its own - no timer backdating
      // needed here (a prior version tried to backdate autoStateStart
      // with an unsigned subtraction that underflowed and cut escape
      // turns short instead of lengthening them - removed).
      float chosenTurn = (autoDistL > autoDistR) ? -AUTO_TURN_SPEED : AUTO_TURN_SPEED;
      leftOut  = slewToward(leftOut,  chosenTurn, dt);
      rightOut = slewToward(rightOut, -chosenTurn, dt);
      applyOutputs(leftOut, rightOut);

      autoStateStart = now;
      autoState = AUTO_TURN;
      break;
    }

    case AUTO_TURN: {
      float turnDirection = (autoDistL > autoDistR) ? -AUTO_TURN_SPEED : AUTO_TURN_SPEED;
      leftOut  = slewToward(leftOut,  turnDirection, dt);
      rightOut = slewToward(rightOut, -turnDirection, dt);
      applyOutputs(leftOut, rightOut);

      bool bothTight = (autoDistL < OBSTACLE_NEAR_CM) && (autoDistR < OBSTACLE_NEAR_CM);
      uint32_t requiredTurnTime = bothTight ? AUTO_ESCAPE_TURN_MS : AUTO_TURN_MS;

      if (now - autoStateStart >= requiredTurnTime) autoState = AUTO_FORWARD;
      break;
    }
  }
}


// =====================================================================
// MODE SWITCH BUTTON
// =====================================================================

static void onModeChanged() {
  stopMotors();
  motorsStopped = true;
  freshPacketSinceModeSwitch = false;  // require a fresh packet before GESTURE drives again
  radarServo.write(90);
  lastRadarPingMs = 0;
  autoState = AUTO_FORWARD;
  autoStateStart = millis();

  Serial.print(F("[MODE] "));
  Serial.println(currentMode == MODE_GESTURE ? F("GESTURE") : F("AUTO"));
}

static void checkModeButton() {
  uint32_t now = millis();
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) lastDebounceMs = now;

  if ((now - lastDebounceMs) > BUTTON_DEBOUNCE_MS && reading != buttonState) {
    buttonState = reading;
    if (buttonState == LOW) {  // pressed, active-low w/ pull-up
      currentMode = (Mode)((currentMode + 1) % MODE_COUNT);
      onModeChanged();
    }
  }
  lastButtonReading = reading;
}


// =====================================================================
// STATUS LED
// =====================================================================

static void updateStatusLed(uint32_t now) {
  bool linkOK = freshPacketSinceModeSwitch && ((now - lastDataMs) <= LINK_TIMEOUT_MS);

  if (currentMode == MODE_GESTURE) {
    if (linkOK) { digitalWrite(LED_BUILTIN, HIGH); return; }
    if (now - lastBlinkMs >= 150) {  // fast blink = no controller link
      lastBlinkMs = now; ledOn = !ledOn;
      digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
    }
    return;
  }

  if (now - lastBlinkMs >= 700) {  // slow blink = AUTO
    lastBlinkMs = now; ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }
}


// =====================================================================
// SERIAL STATUS (debug only - safe to remove for the final build)
// =====================================================================

static void printStatus(uint32_t now) {
  static uint32_t lastPrint = 0;
  if (now - lastPrint < 1000) return;
  lastPrint = now;

  Serial.print(F("[STATUS] mode="));
  Serial.print(currentMode == MODE_GESTURE ? F("GESTURE") : F("AUTO"));
  Serial.print(F("  dist=")); Serial.print(lastRadarCm, 1); Serial.print(F("cm"));
  Serial.print(F("  L=")); Serial.print(leftOut, 2);
  Serial.print(F(" R=")); Serial.print(rightOut, 2);
  Serial.print(F(" link="));
  Serial.println((freshPacketSinceModeSwitch && (now - lastDataMs <= LINK_TIMEOUT_MS)) ? F("OK") : F("LOST"));
}


// =====================================================================
// SETUP
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LEFT_IN1, OUTPUT);  pinMode(LEFT_IN2, OUTPUT);  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT); pinMode(RIGHT_EN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  analogWriteFreq(PWM_FREQ_HZ);
  stopMotors();

  radarServo.attach(SERVO_PIN);
  radarServo.write(90);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F(" XLR8 FINAL COMPETITION BOT - Pico W"));
  Serial.println(F("=========================================="));
  Serial.println(F("Mode: GESTURE"));

  if (!WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL)) {
    Serial.println(F("[WIFI] SoftAP FAILED - password must be 8+ chars"));
  } else {
    Serial.println(F("[WIFI] SoftAP started"));
  }
  while (WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) delay(100);

  Serial.print(F("[WIFI] SSID: ")); Serial.println(AP_SSID);
  Serial.print(F("[WIFI] IP:   ")); Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);
  Serial.print(F("[UDP] Port: ")); Serial.println(UDP_PORT);

  runCalibration();

  uint32_t now = millis();
  lastDataMs = now; lastLoopMs = now; lastRadarPingMs = now; lastBlinkMs = now;

  Serial.println();
  Serial.println(F("[READY] Robot is SAFE."));
  Serial.println(F("[READY] Button: GESTURE <-> AUTO"));
  Serial.println(F("[READY] Waiting for fresh IMU packet..."));
  Serial.println();
}


// =====================================================================
// MAIN LOOP
// =====================================================================

void loop() {
  uint32_t now = millis();
  float dt = (now - lastLoopMs) * 0.001f;
  // Guard against zero dt, a long blocking ultrasonic ping, or a
  // millis() rollover artifact stretching one iteration.
  if (dt <= 0.0f || dt > 0.08f) dt = 0.01f;
  lastLoopMs = now;

  checkModeButton();
  drainPackets();

  switch (currentMode) {
    case MODE_GESTURE: runGesture(now, dt); break;
    case MODE_AUTO:     runAuto(now, dt); break;
    default:            stopMotors(); break;
  }

  updateStatusLed(now);
  printStatus(now);
}
