// ── Motor 1 (Left) ──
#define M1_EN  25
#define M1_FR  26
#define M1_SV  27

// ── Motor 2 (Right) ──
#define M2_EN  14
#define M2_FR  12
#define M2_SV  13

// ── Motor 3 (Left) ──
#define M3_EN  4
#define M3_FR  5
#define M3_SV  18

// ── Motor 4 (Right) ──
#define M4_EN  19
#define M4_FR  21
#define M4_SV  22

// ── Motor 5 (Left) ──
#define M5_EN  23
#define M5_FR  32
#define M5_SV  33

// ── Motor 6 (Right) ──
#define M6_EN  2
#define M6_FR  16
#define M6_SV  15

#define PWM_FREQ 5000
#define PWM_RES  8

// Left: 0,1,2 | Right: 3,4,5 (0-indexed)
int enPins[6]  = {M1_EN, M2_EN, M3_EN, M4_EN, M5_EN, M6_EN};
int frPins[6]  = {M1_FR, M2_FR, M3_FR, M4_FR, M5_FR, M6_FR};
int svPins[6]  = {M1_SV, M2_SV, M3_SV, M4_SV, M5_SV, M6_SV};

// true = motor physically needs FR flipped to match rover direction
bool reversed[6] = {false, true, false, true, false, true};
//                   M1     M2    M3     M4    M5     M6

int globalSpeed = 150;

// ── Helpers ──
void enableAll()  { for (int i=0;i<6;i++) digitalWrite(enPins[i], LOW);  }
void disableAll() { for (int i=0;i<6;i++) digitalWrite(enPins[i], HIGH); }

void setMotorDir(int i, bool fwd) {
  bool dir = reversed[i] ? !fwd : fwd;
  digitalWrite(frPins[i], dir ? HIGH : LOW);
}

void setMotorSpeed(int i, int spd) {
  ledcWrite(svPins[i], constrain(spd, 0, 255));
}

// Set left and right sides simultaneously
void drive(bool leftFwd, int leftSpd, bool rightFwd, int rightSpd) {
  // Set directions first
  for (int i=0;i<3;i++) setMotorDir(i, leftFwd);
  for (int i=3;i<6;i++) setMotorDir(i, rightFwd);
  // Set speeds simultaneously
  for (int i=0;i<3;i++) setMotorSpeed(i, leftSpd);
  for (int i=3;i<6;i++) setMotorSpeed(i, rightSpd);
}

void stopAll() {
  for (int i=0;i<6;i++) setMotorSpeed(i, 0);
}

void setup() {
  // Force EN HIGH immediately on boot
  int enList[6] = {25, 14, 4, 19, 23, 2};
  for (int i=0;i<6;i++) { pinMode(enList[i], OUTPUT); digitalWrite(enList[i], HIGH); }

  Serial.begin(115200);

  for (int i=0;i<6;i++) {
    pinMode(enPins[i], OUTPUT);
    pinMode(frPins[i], OUTPUT);
    digitalWrite(enPins[i], HIGH);
    setMotorDir(i, true);
    ledcAttach(svPins[i], PWM_FREQ, PWM_RES);
    setMotorSpeed(i, 0);
  }

  Serial.println("=== LAKBAY Rover ===");
  Serial.println("  E       — enable motors");
  Serial.println("  Q       — disable motors");
  Serial.println("  W       — forward");
  Serial.println("  X       — backward");
  Serial.println("  A       — turn left");
  Serial.println("  D       — turn right");
  Serial.println("  S       — stop");
  Serial.println("  v150    — set speed (0-255)");
  Serial.println("  +/-     — speed up/down");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "E" || cmd == "e") {
      enableAll();
      Serial.println("Motors ON");

    } else if (cmd == "Q" || cmd == "q") {
      stopAll();
      disableAll();
      Serial.println("Motors OFF");

    } else if (cmd == "W" || cmd == "w") {
      drive(true, globalSpeed, true, globalSpeed);
      Serial.print("FORWARD  spd="); Serial.println(globalSpeed);

    } else if (cmd == "X" || cmd == "x") {
      drive(false, globalSpeed, false, globalSpeed);
      Serial.print("BACKWARD  spd="); Serial.println(globalSpeed);

    } else if (cmd == "A" || cmd == "a") {
      // Left: reverse | Right: forward
      drive(false, globalSpeed, true, globalSpeed);
      Serial.print("LEFT  spd="); Serial.println(globalSpeed);

    } else if (cmd == "D" || cmd == "d") {
      // Left: forward | Right: reverse
      drive(true, globalSpeed, false, globalSpeed);
      Serial.print("RIGHT  spd="); Serial.println(globalSpeed);

    } else if (cmd == "S" || cmd == "s") {
      stopAll();
      Serial.println("STOP");

    } else if (cmd == "+" || cmd == "=") {
      globalSpeed = constrain(globalSpeed + 10, 0, 255);
      Serial.print("Speed: "); Serial.println(globalSpeed);

    } else if (cmd == "-" || cmd == "_") {
      globalSpeed = constrain(globalSpeed - 10, 0, 255);
      Serial.print("Speed: "); Serial.println(globalSpeed);

    } else if (cmd.charAt(0) == 'v' || cmd.charAt(0) == 'V') {
      globalSpeed = constrain(cmd.substring(1).toInt(), 0, 255);
      Serial.print("Speed set: "); Serial.println(globalSpeed);
    }
  }
}
