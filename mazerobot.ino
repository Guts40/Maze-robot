/*
   Maze Solving Robot - Right Hand Wall Follower with IR Safety
   Arduino UNO + L298N + 3 HC-SR04 + 2 IR Sensors
   Enhanced logic combining ultrasonic navigation and IR close-range safety
*/

#define TRIG_LEFT   A5
#define ECHO_LEFT   A4
#define TRIG_FRONT  A3
#define ECHO_FRONT  A2
#define TRIG_RIGHT  A1
#define ECHO_RIGHT  A0

#define LEFT_IR     3   // Digital IR Left
#define RIGHT_IR    4   // Digital IR Right

// L298N Motor Driver Pins
#define IN1         12
#define IN2         11
#define ENA         7   // PWM Left
#define IN3         10
#define IN4         9
#define ENB         8   // PWM Right

const int BASE_SPEED     = 180;  // Forward speed (0-255)
const int TURN_SPEED     = 160;  // Turn speed
const int OPEN_THRESHOLD = 25;   // cm - open path if > 25 cm
const int PWM_VALUE      = 180;  // Main motor PWM

void setup() {
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(TRIG_LEFT,  OUTPUT);
  pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // IR sensors
  pinMode(LEFT_IR,  INPUT);
  pinMode(RIGHT_IR, INPUT);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopRobot();
  delay(2000);
}

// Ultrasonic distance measurement (returns cm)
int getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);  // Timeout 30ms
  if (duration == 0) return 500;  // Far/open

  return (int)(duration / 29 / 2);
}

// Motor movement functions
void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, PWM_VALUE);
  analogWrite(ENB, PWM_VALUE);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void uTurn() {
  turnLeft();
  delay(700);  // Adjust for 180° turn
}

void slightRight() {
  analogWrite(ENA, BASE_SPEED + 60);
  analogWrite(ENB, BASE_SPEED - 60);
  forward();
}

void slightLeft() {
  analogWrite(ENA, BASE_SPEED - 60);
  analogWrite(ENB, BASE_SPEED + 60);
  forward();
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  int leftDist  = getDistance(TRIG_LEFT,  ECHO_LEFT);
  int frontDist = getDistance(TRIG_FRONT, ECHO_FRONT);
  int rightDist = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  int leftIR  = digitalRead(LEFT_IR);   // 1 = obstacle detected
  int rightIR = digitalRead(RIGHT_IR);

  // === IR Close-Range Safety Override (Priority 1) ===
  if (leftIR == 1 && rightIR == 1) {
    stopRobot();
    delay(300);
    uTurn();
    delay(300);
    return;
  }
  else if (leftIR == 1) {
    slightRight();    // Too close on left → steer right
    delay(150);
    return;
  }
  else if (rightIR == 1) {
    slightLeft();     // Too close on right → steer left
    delay(150);
    return;
  }

  // === Ultrasonic Right-Hand Rule Navigation (Priority 2) ===
  if (rightDist > OPEN_THRESHOLD) {
    turnRight();
    delay(400);
    while (getDistance(TRIG_FRONT, ECHO_FRONT) <= OPEN_THRESHOLD) {
      turnRight();  // Keep turning until front clear
    }
  }
  else if (frontDist > OPEN_THRESHOLD) {
    forward();
  }
  else if (leftDist > OPEN_THRESHOLD) {
    turnLeft();
    delay(400);
    while (getDistance(TRIG_FRONT, ECHO_FRONT) <= OPEN_THRESHOLD) {
      turnLeft();
    }
  }
  else {
    uTurn();  // Dead end
  }

  // Debug output
  Serial.print("L: "); Serial.print(leftDist);
  Serial.print(" F: "); Serial.print(frontDist);
  Serial.print(" R: "); Serial.print(rightDist);
  Serial.print(" | IR_L: "); Serial.print(leftIR);
  Serial.print(" IR_R: "); Serial.println(rightIR);

  delay(50);
}