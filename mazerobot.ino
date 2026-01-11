// Maze Solving Robot - Left Hand Rule with 3 Ultrasonic + 2 IR
// Competition Grade - Smooth PWM - No delay() lag
// Fixed pins - Arduino UNO R3

// ---------------- Ultrasonic Pins ----------------
#define TRIG_LEFT   A5
#define ECHO_LEFT   A4
#define TRIG_CENTER A3
#define ECHO_CENTER A2
#define TRIG_RIGHT  A1
#define ECHO_RIGHT  A0

// ---------------- IR Pins ----------------
#define IR_LEFT  2
#define IR_RIGHT 4

// ---------------- Motor Pins (L298N) ----------------
#define ENA 6   // PWM Left motors
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENB 11  // PWM Right motors

// ---------------- Thresholds (cm) ----------------
const int WALL_THRESHOLD    = 25;   // Wall detected if < this
const int FRONT_SAFE        = 18;   // Critical front distance
const int TURN_SPEED        = 130;  // 0-255
const int CRUISE_SPEED      = 150;
const int ALIGN_SPEED_DIFF  = 40;   // For wall hugging correction

// IR logic (adjust according to your module: LOW = wall detected or HIGH)
const bool IR_DETECTS_WALL = LOW;   // Change to HIGH if your module outputs HIGH on detection

void setup() {
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(TRIG_LEFT,   OUTPUT);
  pinMode(ECHO_LEFT,   INPUT);
  pinMode(TRIG_CENTER, OUTPUT);
  pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_RIGHT,  OUTPUT);
  pinMode(ECHO_RIGHT,  INPUT);

  // IR pins
  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();  // Safety start
  delay(2000);   // Wait for stability
}

void loop() {
  int distLeft   = getDistance(TRIG_LEFT,   ECHO_LEFT);
  int distCenter = getDistance(TRIG_CENTER, ECHO_CENTER);
  int distRight  = getDistance(TRIG_RIGHT,  ECHO_RIGHT);

  bool irLeftWall  = (digitalRead(IR_LEFT)  == IR_DETECTS_WALL);
  bool irRightWall = (digitalRead(IR_RIGHT) == IR_DETECTS_WALL);

  // Emergency collision check
  if (distCenter < 10 || irLeftWall || irRightWall) {
    stopMotors();
    reverse(80, 300);   // Back off a bit
    turnAround();
    return;
  }

  // ---------------- Left-Hand Rule Logic ----------------
  if (distLeft > WALL_THRESHOLD) {
    // Left is open → Priority: TURN LEFT
    smoothLeftTurn();
  }
  else if (distCenter > FRONT_SAFE) {
    // Front open → GO STRAIGHT + wall alignment correction
    forwardWithAlignment(distLeft, distRight, irLeftWall, irRightWall);
  }
  else if (distRight > WALL_THRESHOLD) {
    // Right open → TURN RIGHT
    smoothRightTurn();
  }
  else {
    // Dead end → TURN AROUND
    turnAround();
  }
}

// ---------------- Distance Function (Non-blocking style) ----------------
int getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);  // Timeout ~5m
  if (duration == 0) return 999;               // Error → far
  int cm = duration / 58;
  return cm;
}

// ---------------- Motor Control Functions ----------------
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motors
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(ENA, leftSpeed);

  // Right motors
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENB, rightSpeed);
}

void forward(int speed) {
  setMotors(speed, speed);
}

void stopMotors() {
  setMotors(0, 0);
}

void reverse(int speed, int ms) {
  setMotors(-speed, -speed);
  delay(ms);   // Short controlled reverse only
  stopMotors();
}

void smoothLeftTurn() {
  forwardWithSpeed(CRUISE_SPEED);
  delay(100); // Small advance
  setMotors(-TURN_SPEED, TURN_SPEED);
  delay(380); // Adjust for ~90° - test & tune
  stopMotors();
  delay(50);
}

void smoothRightTurn() {
  forwardWithSpeed(CRUISE_SPEED);
  delay(100);
  setMotors(TURN_SPEED, -TURN_SPEED);
  delay(380); // Adjust
  stopMotors();
  delay(50);
}

void turnAround() {
  setMotors(TURN_SPEED, -TURN_SPEED);
  delay(750); // ~180° - tune
  stopMotors();
  delay(100);
}

void forwardWithSpeed(int speed) {
  setMotors(speed, speed);
}

// Wall hugging correction during straight motion
void forwardWithAlignment(int dLeft, int dRight, bool irL, bool irR) {
  int correction = 0;

  // Prefer left-hand wall, but use right if left missing
  if (dLeft < WALL_THRESHOLD - 5) {
    correction += ALIGN_SPEED_DIFF;   // too close to left wall → slow left motor
  } 
  else if (dLeft > WALL_THRESHOLD + 5 && dLeft < 999) {
    correction -= ALIGN_SPEED_DIFF;   // too far from left wall → speed up left motor
  }
  else if (dRight < WALL_THRESHOLD - 5) {
    correction -= ALIGN_SPEED_DIFF;   // too close to right wall → slow right motor
  }

  int leftSpeed  = CRUISE_SPEED + correction;
  int rightSpeed = CRUISE_SPEED - correction;

  setMotors(leftSpeed, rightSpeed);
}

  if (dLeft < WALL_THRESHOLD - 5) correction += ALIGN_SPEED_DIFF;     // Too close left → slow left
  if (dLeft > WALL_THRESHOLD + 5) correction -= ALIGN_SPEED_DIFF;     // Too far left → speed up left

  forwardWithSpeed(CRUISE_SPEED + correction);
}
