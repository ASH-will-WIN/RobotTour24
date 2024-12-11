//Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 3200  // 200 * 16 for 1/16 microstepping

#define dirPin2 4
#define stepPin2 5

// Microstepping control pins
#define MS1 6
#define MS2 7
#define MS3 8

#define FORWARD_1 HIGH
#define FORWARD_2 LOW
#define REVERSE_1 LOW
#define REVERSE_2 HIGH

float wheelDiameter = 4.0132 * 2;
float circumference = M_PI * wheelDiameter;
double wheelbase = 15.875;

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Microstepping pins:
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  // Set microstepping to 1/16
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  Serial.begin(115200);
  Serial.println("Code is running with 1/16 microstepping");
  delay(10000);

  // Test movement
  moveForward(25, 5000);
  turnRight(15, 5000);
  delay(1000);
  moveForward(150, 5000);
  delay(1000);
  turnLeft(15, 5000);
  delay(1000);
  moveForward(200, 5000);
  delay(1000);
  turnLeft(15, 5000);
  delay(1000);
  moveForward(150, 5000);
  turnLeft(15, 5000);
  moveForward(150, 5000);
  turnLeft(15, 5000);
  moveForward(150, 5000);
}

void loop() {
  // Empty loop for now
}

float distanceToSteps(int cm) {
  float revolutions = cm / circumference;
  return revolutions * stepsPerRevolution;
}

void move(int distance, int speed) {
  for (int i = 0; i < distanceToSteps(distance); i++) {
    // Step Motor 1
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(speed / 16); // Adjust speed for microstepping
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(speed / 16);
  }
}

void moveForward(int distance, int speed) {
  digitalWrite(dirPin, FORWARD_1);
  digitalWrite(dirPin2, FORWARD_2);
  move(distance, speed);
}

void moveBackward(int distance, int speed) {
  digitalWrite(dirPin, REVERSE_1);
  digitalWrite(dirPin2, REVERSE_2);
  move(distance, speed);
}

void turnRight(int distance, int speed) {
  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);
  move(distance, speed);
}

void turnLeft(int distance, int speed) {
  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, LOW);
  move(distance, speed);
}