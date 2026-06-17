/*
 * ============================================================
 *  ESP32 DevKit V1 — Skid-Steer Rover Controller
 *  3× Left + 3× Right DC Motors via 3× L298N
 *  Controlled via Serial JSON from Python
 * ============================================================
 *
 *  REQUIRED LIBRARIES (install via Arduino Library Manager):
 *    - ArduinoJson  by Benoit Blanchon
 *
 * ============================================================
 *  MOTOR LAYOUT
 * ============================================================
 *
 *   LEFT SIDE                RIGHT SIDE
 *   ─────────                ──────────
 *   Motor 1 (front-left)     Motor 4 (front-right)
 *   Motor 2 (mid-left)       Motor 5 (mid-right)
 *   Motor 3 (rear-left)      Motor 6 (rear-right)
 *
 * ============================================================
 *  WIRING DIAGRAM
 * ============================================================
 *
 *  ── L298N #1  (Controls DC Motors 1 & 2) ──
 *    ENA  → GPIO 23     (Motor 1 PWM speed) ← REMOVE jumper cap!
 *    IN1  → GPIO 32     (Motor 1 forward)
 *    IN2  → GPIO 33     (Motor 1 backward)
 *    ENB  → GPIO 22     (Motor 2 PWM speed) ← REMOVE jumper cap!
 *    IN3  → GPIO 25     (Motor 2 forward)
 *    IN4  → GPIO 26     (Motor 2 backward)
 *
 *  ── L298N #2  (Controls DC Motors 3 & 4) ──
 *    ENA  → GPIO 4      (Motor 3 PWM speed) ← REMOVE jumper cap!
 *    IN1  → GPIO 16     (Motor 3 forward)
 *    IN2  → GPIO 17     (Motor 3 backward)
 *    ENB  → GPIO 2      (Motor 4 PWM speed) ← REMOVE jumper cap!
 *    IN3  → GPIO 15     (Motor 4 forward)
 *    IN4  → GPIO 13     (Motor 4 backward)
 *
 *  ── L298N #3  (Controls DC Motors 5 & 6) ──
 *    ENA  → GPIO 12     (Motor 5 PWM speed) ← REMOVE jumper cap!
 *    IN1  → GPIO 14     (Motor 5 forward)
 *    IN2  → GPIO 27     (Motor 5 backward)
 *    ENB  → GPIO 0      (Motor 6 PWM speed) ← REMOVE jumper cap!
 *    IN3  → GPIO 18     (Motor 6 forward)
 *    IN4  → GPIO 5      (Motor 6 backward)
 *
 *  ⚠️  GPIO 2  is the onboard LED; it will blink when Motor 4 runs.
 *  ⚠️  GPIO 0  is the boot pin. Motor 6 PWM may cause boot issues if
 *      held LOW at power-on. Consider remapping ENB of L298N #3 if
 *      the board fails to boot with motors connected.
 *  ⚠️  All GNDs (ESP32, all L298N boards) MUST share a common ground.
 *
 * ============================================================
 *  PIN SUMMARY
 * ============================================================
 *
 *   GPIO  0 — Motor 6 PWM (ENB, L298N #3)   ⚠️ boot pin
 *   GPIO  2 — Motor 4 PWM (ENB, L298N #2)   ⚠️ onboard LED
 *   GPIO  4 — Motor 3 PWM (ENA, L298N #2)
 *   GPIO  5 — Motor 6 DIR (IN4, L298N #3)
 *   GPIO 12 — Motor 5 PWM (ENA, L298N #3)
 *   GPIO 13 — Motor 4 DIR (IN4, L298N #2)
 *   GPIO 14 — Motor 5 DIR (IN1, L298N #3)
 *   GPIO 15 — Motor 4 DIR (IN3, L298N #2)
 *   GPIO 16 — Motor 3 DIR (IN1, L298N #2)
 *   GPIO 17 — Motor 3 DIR (IN2, L298N #2)
 *   GPIO 18 — Motor 6 DIR (IN3, L298N #3)
 *   GPIO 22 — Motor 2 PWM (ENB, L298N #1)
 *   GPIO 23 — Motor 1 PWM (ENA, L298N #1)
 *   GPIO 25 — Motor 2 DIR (IN3, L298N #1)
 *   GPIO 26 — Motor 2 DIR (IN4, L298N #1)
 *   GPIO 27 — Motor 5 DIR (IN2, L298N #3)
 *   GPIO 32 — Motor 1 DIR (IN1, L298N #1)
 *   GPIO 33 — Motor 1 DIR (IN2, L298N #1)
 *
 * ============================================================
 *  SERIAL COMMAND PROTOCOL  (115200 baud, newline-terminated)
 * ============================================================
 *
 *  Drive forward:
 *    {"cmd":"forward","speed":200}
 *      speed : 0–255
 *
 *  Drive backward:
 *    {"cmd":"backward","speed":200}
 *      speed : 0–255
 *
 *  Turn left (skid-steer — left side reverses, right side forwards):
 *    {"cmd":"left","speed":200}
 *      speed : 0–255
 *
 *  Turn right (skid-steer — right side reverses, left side forwards):
 *    {"cmd":"right","speed":200}
 *      speed : 0–255
 *
 *  Set individual DC motor speed / direction:
 *    {"cmd":"motor","id":1,"speed":200}
 *      id    : 1–6  (which DC motor)
 *      speed : -255 to 255  (negative = reverse, 0 = stop)
 *
 *  Stop all DC motors:
 *    {"cmd":"stop"}
 *
 *  Get full status:
 *    {"cmd":"status"}
 */

#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════
//  DC MOTOR CONFIGURATION
// ═══════════════════════════════════════════════════════════════
#define PWM_FREQ        1000   // 1 kHz
#define PWM_RESOLUTION     8   // 8-bit → 0–255
#define NUM_MOTORS         6

struct MotorPins {
  uint8_t en;   // PWM speed (ENA or ENB — remove jumper cap!)
  uint8_t in1;  // direction pin A
  uint8_t in2;  // direction pin B
};

const MotorPins MOTORS[NUM_MOTORS] = {
  //  EN   IN1  IN2
  {  23,  32,  33 },  // Motor 1 — LEFT  front  (L298N #1 Channel A)
  {  22,  25,  26 },  // Motor 2 — LEFT  mid    (L298N #1 Channel B)
  {   4,  16,  17 },  // Motor 3 — LEFT  rear   (L298N #2 Channel A)
  {   2,  15,  13 },  // Motor 4 — RIGHT front  (L298N #2 Channel B)
  {  12,  14,  27 },  // Motor 5 — RIGHT mid    (L298N #3 Channel A)
  {   0,  18,   5 },  // Motor 6 — RIGHT rear   (L298N #3 Channel B)  ⚠️ GPIO 0
};

// Motors 1-3 = LEFT side, Motors 4-6 = RIGHT side
#define LEFT_MOTORS_START  0   // index 0,1,2
#define RIGHT_MOTORS_START 3   // index 3,4,5

int motorSpeeds[NUM_MOTORS] = { 0 };

// ═══════════════════════════════════════════════════════════════
//  DC MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════
void setMotor(int idx, int speed) {
  speed = constrain(speed, -255, 255);
  motorSpeeds[idx] = speed;

  uint8_t en       = MOTORS[idx].en;
  uint8_t in1      = MOTORS[idx].in1;
  uint8_t in2      = MOTORS[idx].in2;
  int     absSpeed = abs(speed);

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
  ledcWrite(en, absSpeed);
}

void stopAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) setMotor(i, 0);
}

// Drive left side and right side at given speeds (positive = forward)
void setLeftRight(int leftSpeed, int rightSpeed) {
  for (int i = LEFT_MOTORS_START;  i < LEFT_MOTORS_START  + 3; i++) setMotor(i, leftSpeed);
  for (int i = RIGHT_MOTORS_START; i < RIGHT_MOTORS_START + 3; i++) setMotor(i, rightSpeed);
}

// ═══════════════════════════════════════════════════════════════
//  JSON COMMAND PROCESSOR
// ═══════════════════════════════════════════════════════════════
void processCommand(const String& input) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, input);

  if (err) {
    Serial.println(F("{\"status\":\"error\",\"message\":\"Invalid JSON\"}"));
    return;
  }

  const char* cmd = doc["cmd"];
  if (!cmd) {
    Serial.println(F("{\"status\":\"error\",\"message\":\"Missing 'cmd' field\"}"));
    return;
  }

  // ── forward ────────────────────────────────────────────────
  if (strcmp(cmd, "forward") == 0) {
    int speed = doc["speed"] | 200;
    speed = constrain(speed, 0, 255);
    setLeftRight(speed, speed);
    StaticJsonDocument<128> resp;
    resp["status"] = "ok";
    resp["cmd"]    = "forward";
    resp["speed"]  = speed;
    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── backward ───────────────────────────────────────────────
  else if (strcmp(cmd, "backward") == 0) {
    int speed = doc["speed"] | 200;
    speed = constrain(speed, 0, 255);
    setLeftRight(-speed, -speed);
    StaticJsonDocument<128> resp;
    resp["status"] = "ok";
    resp["cmd"]    = "backward";
    resp["speed"]  = speed;
    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── turn left (skid-steer) ─────────────────────────────────
  //    Left side reverses, right side drives forward
  else if (strcmp(cmd, "left") == 0) {
    int speed = doc["speed"] | 200;
    speed = constrain(speed, 0, 255);
    setLeftRight(-speed, speed);
    StaticJsonDocument<128> resp;
    resp["status"] = "ok";
    resp["cmd"]    = "left";
    resp["speed"]  = speed;
    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── turn right (skid-steer) ────────────────────────────────
  //    Right side reverses, left side drives forward
  else if (strcmp(cmd, "right") == 0) {
    int speed = doc["speed"] | 200;
    speed = constrain(speed, 0, 255);
    setLeftRight(speed, -speed);
    StaticJsonDocument<128> resp;
    resp["status"] = "ok";
    resp["cmd"]    = "right";
    resp["speed"]  = speed;
    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── individual motor ───────────────────────────────────────
  else if (strcmp(cmd, "motor") == 0) {
    int id    = doc["id"]    | 0;
    int speed = doc["speed"] | 0;

    if (id < 1 || id > NUM_MOTORS) {
      Serial.println(F("{\"status\":\"error\",\"message\":\"Motor id must be 1-6\"}"));
      return;
    }
    setMotor(id - 1, speed);

    StaticJsonDocument<128> resp;
    resp["status"] = "ok";
    resp["motor"]  = id;
    resp["speed"]  = motorSpeeds[id - 1];
    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── stop all DC motors ──────────────────────────────────────
  else if (strcmp(cmd, "stop") == 0) {
    stopAllMotors();
    Serial.println(F("{\"status\":\"ok\",\"message\":\"All motors stopped\"}"));
  }

  // ── full status ─────────────────────────────────────────────
  else if (strcmp(cmd, "status") == 0) {
    StaticJsonDocument<512> resp;
    resp["status"] = "ok";

    JsonArray mArr = resp.createNestedArray("motors");
    for (int i = 0; i < NUM_MOTORS; i++) mArr.add(motorSpeeds[i]);

    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── unknown command ─────────────────────────────────────────
  else {
    Serial.println(F("{\"status\":\"error\",\"message\":\"Unknown command\"}"));
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(0);

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (MOTORS[i].in1 < 34) { pinMode(MOTORS[i].in1, OUTPUT); digitalWrite(MOTORS[i].in1, LOW); }
    if (MOTORS[i].in2 < 34) { pinMode(MOTORS[i].in2, OUTPUT); digitalWrite(MOTORS[i].in2, LOW); }

    ledcAttach(MOTORS[i].en, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTORS[i].en, 0);
  }

  Serial.println(F("{\"status\":\"ready\",\"message\":\"ESP32 Skid-Steer Rover Online — 6 DC Motors (3L + 3R)\"}"));
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      processCommand(line);
    }
  }
}
