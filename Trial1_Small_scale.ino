/*
 * ============================================================
 *  ESP32 DevKit V1 — Motor Controller
 *  3× L298N (6 DC motors) + 4× MG996R Servo Motors
 *  Controlled via Serial JSON from Python
 * ============================================================
 *
 *  REQUIRED LIBRARIES (install via Arduino Library Manager):
 *    - ESP32Servo   by Kevin Harrington
 *    - ArduinoJson  by Benoit Blanchon
 *
 * ============================================================
 *  WIRING DIAGRAM
 * ============================================================
 *
 *  L298N #1  (Controls DC Motors 1 & 2)
 *    ENA  → GPIO 23     (Motor 1 PWM speed) ← REMOVE jumper cap!
 *    IN1  → GPIO 32     (Motor 1 forward)
 *    IN2  → GPIO 33     (Motor 1 backward)
 *    ENB  → GPIO 22     (Motor 2 PWM speed) ← REMOVE jumper cap!
 *    IN3  → GPIO 25     (Motor 2 forward)
 *    IN4  → GPIO 26     (Motor 2 backward)
 *
 *  L298N #2  (Controls DC Motors 3 & 4)
 *    ENA  → GPIO 4      (Motor 3 PWM speed) ← REMOVE jumper cap!
 *    IN1  → GPIO 16     (Motor 3 forward)
 *    IN2  → GPIO 17     (Motor 3 backward)
 *    ENB  → GPIO 2      (Motor 4 PWM speed) ← REMOVE jumper cap!
 *    IN3  → GPIO 15     (Motor 4 forward)
 *    IN4  → GPIO 13     (Motor 4 backward)
 *
 *  L298N #3  (Controls DC Motors 5 & 6)
 *    ENA  → GPIO 12     (Motor 5 PWM speed) ← REMOVE jumper cap!
 *    IN1  → GPIO 14     (Motor 5 forward)
 *    IN2  → GPIO 27     (Motor 5 backward)
 *    ENB  → GPIO 0      (Motor 6 PWM speed) ← REMOVE jumper cap!
 *    IN3  → GPIO 34     (Motor 6 forward)   ← INPUT ONLY — use 35 if issues
 *    IN4  → GPIO 35     (Motor 6 backward)  ← INPUT ONLY — use 39 if issues
 *
 *  ⚠️  GPIO 34/35 are INPUT-ONLY on ESP32. If Motor 6 direction
 *      control does not work, remap IN3/IN4 to available output GPIOs.
 *      Suggested alternatives: GPIO 36→swap with a free output pin.
 *      Safe free output-capable GPIOs: 14, 27 (already used above),
 *      consider reducing to 5 motors or using an I/O expander.
 *
 *  ⚠️  IMPORTANT: Remove the yellow ENA/ENB jumper caps from ALL L298N
 *      boards before wiring PWM pins, otherwise you'll short the GPIO!
 *
 *  MG996R Servos (Signal wire only; power from 5V external supply)
 *    Servo 1 Signal → GPIO 5
 *    Servo 2 Signal → GPIO 18
 *    Servo 3 Signal → GPIO 19
 *    Servo 4 Signal → GPIO 21
 *    All Servo GND  → ESP32 GND & external supply GND (common ground!)
 *
 *  ⚠️  IMPORTANT NOTES:
 *    - MG996R servos draw 500mA–2.5A each. Use an external 5V/6V power
 *      supply rated for at least 10A for all 4 servos. Do NOT power
 *      servos from the ESP32 5V pin.
 *    - L298N motor power (VM) should match your DC motor voltage (6–12V).
 *    - All GNDs (ESP32, L298N ×3, servo supply) MUST share a common ground.
 *    - GPIO 2 is the onboard LED pin; it will blink when Motor 4 runs.
 *    - GPIO 0 is the boot pin; Motor 6 PWM may cause boot issues if held
 *      LOW at power-on. Consider remapping ENB of L298N #3 if unstable.
 *
 * ============================================================
 *  SERIAL COMMAND PROTOCOL  (115200 baud, newline-terminated)
 * ============================================================
 *
 *  Set DC motor speed/direction:
 *    {"cmd":"motor","id":1,"speed":200}
 *      id    : 1–6  (which DC motor)
 *      speed : -255 to 255 (negative = reverse, 0 = stop)
 *
 *  Set servo angle:
 *    {"cmd":"servo","id":1,"angle":90}
 *      id    : 1–4  (which servo)
 *      angle : 0–180 degrees
 *
 *  Stop all DC motors:
 *    {"cmd":"stop"}
 *
 *  Center all servos:
 *    {"cmd":"center"}
 *
 *  Get status:
 *    {"cmd":"status"}
 */

#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ─── DC Motor Pin Definitions ──────────────────────────────────────────────
#define PWM_FREQ       1000   // 1 kHz
#define PWM_RESOLUTION    8   // 8-bit → 0–255
#define NUM_MOTORS        6

struct MotorPins {
  uint8_t en;   // ENA or ENB — PWM speed pin (remove jumper cap!)
  uint8_t in1;  // direction pin A
  uint8_t in2;  // direction pin B
};

const MotorPins MOTORS[NUM_MOTORS] = {
  //  EN   IN1  IN2
  {  23,  32,  33 },  // Motor 1 — L298N #1 Channel A
  {  22,  25,  26 },  // Motor 2 — L298N #1 Channel B
  {   4,  16,  17 },  // Motor 3 — L298N #2 Channel A
  {   2,  15,  13 },  // Motor 4 — L298N #2 Channel B
  {  12,  14,  27 },  // Motor 5 — L298N #3 Channel A
  {   0,  34,  35 },  // Motor 6 — L298N #3 Channel B ⚠️ see GPIO notes above
};

// ─── Servo Pin Definitions ─────────────────────────────────────────────────
#define NUM_SERVOS 4
const uint8_t SERVO_PINS[NUM_SERVOS] = {5, 18, 19, 21};

#define SERVO_MIN_US   500
#define SERVO_MAX_US  2400

// ─── State Tracking ────────────────────────────────────────────────────────
Servo servos[NUM_SERVOS];
int   motorSpeeds[NUM_MOTORS] = {0};
int   servoAngles[NUM_SERVOS] = {90, 90, 90, 90};

// ──────────────────────────────────────────────────────────────────────────
//  Motor Control
// ──────────────────────────────────────────────────────────────────────────
void setMotor(int idx, int speed) {
  speed = constrain(speed, -255, 255);
  motorSpeeds[idx] = speed;

  uint8_t en  = MOTORS[idx].en;
  uint8_t in1 = MOTORS[idx].in1;
  uint8_t in2 = MOTORS[idx].in2;
  int absSpeed = abs(speed);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(en, absSpeed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(en, absSpeed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(en, 0);
  }
}

void stopAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) setMotor(i, 0);
}

// ──────────────────────────────────────────────────────────────────────────
//  Servo Control
// ──────────────────────────────────────────────────────────────────────────
void setServo(int idx, int angle) {
  angle = constrain(angle, 0, 180);
  servoAngles[idx] = angle;
  servos[idx].write(angle);
}

void centerAllServos() {
  for (int i = 0; i < NUM_SERVOS; i++) setServo(i, 90);
}

// ──────────────────────────────────────────────────────────────────────────
//  JSON Command Processor
// ──────────────────────────────────────────────────────────────────────────
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

  // ── motor ──────────────────────────────────────────────
  if (strcmp(cmd, "motor") == 0) {
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

  // ── servo ──────────────────────────────────────────────
  else if (strcmp(cmd, "servo") == 0) {
    int id    = doc["id"]    | 0;
    int angle = doc["angle"] | 90;

    if (id < 1 || id > NUM_SERVOS) {
      Serial.println(F("{\"status\":\"error\",\"message\":\"Servo id must be 1-4\"}"));
      return;
    }
    setServo(id - 1, angle);
    StaticJsonDocument<128> resp;
    resp["status"] = "ok";
    resp["servo"]  = id;
    resp["angle"]  = servoAngles[id - 1];
    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── stop all DC motors ─────────────────────────────────
  else if (strcmp(cmd, "stop") == 0) {
    stopAllMotors();
    Serial.println(F("{\"status\":\"ok\",\"message\":\"All DC motors stopped\"}"));
  }

  // ── center all servos ──────────────────────────────────
  else if (strcmp(cmd, "center") == 0) {
    centerAllServos();
    Serial.println(F("{\"status\":\"ok\",\"message\":\"All servos centered at 90\"}"));
  }

  // ── status report ──────────────────────────────────────
  else if (strcmp(cmd, "status") == 0) {
    StaticJsonDocument<512> resp;
    resp["status"] = "ok";

    JsonArray mArr = resp.createNestedArray("motors");
    for (int i = 0; i < NUM_MOTORS; i++) mArr.add(motorSpeeds[i]);

    JsonArray sArr = resp.createNestedArray("servos");
    for (int i = 0; i < NUM_SERVOS; i++) sArr.add(servoAngles[i]);

    serializeJson(resp, Serial);
    Serial.println();
  }

  // ── unknown command ────────────────────────────────────
  else {
    Serial.println(F("{\"status\":\"error\",\"message\":\"Unknown command\"}"));
  }
}

// ──────────────────────────────────────────────────────────────────────────
//  Setup
// ──────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // ── DC Motors ────────────────────────────────────────
  for (int i = 0; i < NUM_MOTORS; i++) {
    // GPIO 34 & 35 are input-only — skip pinMode for them
    if (MOTORS[i].in1 < 34) pinMode(MOTORS[i].in1, OUTPUT);
    if (MOTORS[i].in2 < 34) pinMode(MOTORS[i].in2, OUTPUT);
    if (MOTORS[i].in1 < 34) digitalWrite(MOTORS[i].in1, LOW);
    if (MOTORS[i].in2 < 34) digitalWrite(MOTORS[i].in2, LOW);

    ledcAttach(MOTORS[i].en, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTORS[i].en, 0);
  }

  // ── Servos ───────────────────────────────────────────
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    servos[i].write(90);
    servoAngles[i] = 90;
  }

  Serial.println(F("{\"status\":\"ready\",\"message\":\"ESP32 Motor Controller Online — 6 Motors, 4 Servos\"}"));
}

// ──────────────────────────────────────────────────────────────────────────
//  Main Loop
// ──────────────────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      processCommand(line);
    }
  }
}