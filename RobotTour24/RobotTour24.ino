#include <Math.h>

#define DIR_PIN_LEFT   2    // Direction pin for left motor
#define STEP_PIN_LEFT  3    // Step pin for left motor

#define DIR_PIN_RIGHT  4    // Direction pin for right motor
#define STEP_PIN_RIGHT 5    // Step pin for right motor

#define MS1 6
#define MS2 7
#define MS3 8

struct Point {
  double x;
  double y;
};

double lookaheadDistance = 200;
double baseSpeed = 10000;  // Base speed in steps per second
double scaleFactor = 1;

double leftSpeed = 0;
double rightSpeed = 0;

double WHEELBASE = 15.875;  // Distance between wheels in cm
double WHEEL_RADIUS = 4.0132;  // Wheel radius in cm
double WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
double STEPS_PER_REV = 200 * 16;  // 16 microsteps per step
double dStep = WHEEL_CIRCUMFERENCE / STEPS_PER_REV;
double totalDistance = 0;

double initialHeading = 0;
double currentHeading = initialHeading;
Point initialPos = {0, 0};
Point currentPos = {0, 0};
Point desiredPositions[5] = {{0, 0}, {0, 10}, {0, 20}, {0, 30}, {0, 40}};

// Timing variables
unsigned long previousTime = 0;

void setup() {
  Serial.begin(115200);

  // Set motor pin modes
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(STEP_PIN_LEFT, OUTPUT);

  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(STEP_PIN_RIGHT, OUTPUT);


  // Microstepping pins:
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  // Set microstepping to 1/16
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);
}

void loop() {
  updateValues();

  // Calculate curvature and adjust motor speeds
  double curvature = calculateCurvature(findLookaheadPoint());
  adjustMotorSpeeds(curvature);

  // Move the motors at the calculated speeds
  moveMotor(STEP_PIN_LEFT, DIR_PIN_LEFT, leftSpeed);
  moveMotor(STEP_PIN_RIGHT, DIR_PIN_RIGHT, rightSpeed);

  // Stop the motors if path is complete
  if (isPathComplete(desiredPositions[4], 3)) {
    Serial.println("Path complete!");
    while (true); // Stop execution
  }
}

// Move a motor based on speed
void moveMotor(int stepPin, int dirPin, double speed) {
  static unsigned long lastStepTimeLeft = 0;
  static unsigned long lastStepTimeRight = 0;

  unsigned long &lastStepTime = (stepPin == STEP_PIN_LEFT) ? lastStepTimeLeft : lastStepTimeRight;
  unsigned long stepInterval = 1000000.0 / abs(speed); // Step interval in microseconds

  if (micros() - lastStepTime >= stepInterval) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2); // Minimum pulse width for your motor driver
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();
  }
}


// Update robot state (position, heading, etc.)
void updateValues() {
  unsigned long currentTime = millis();
  double deltaTime = (currentTime - previousTime) / 1000.0; // Time in seconds
  previousTime = currentTime;

  double leftDistance = leftSpeed * deltaTime * dStep;
  double rightDistance = rightSpeed * deltaTime * dStep;

  double deltaHeading = (leftDistance - rightDistance) / WHEELBASE;
  currentHeading += deltaHeading;

  double avgDistance = (leftDistance + rightDistance) / 2;
  currentPos.x += avgDistance * cos(currentHeading);
  currentPos.y += avgDistance * sin(currentHeading);

  // Log current pose
  Serial.print("Current Position: (");
  Serial.print(currentPos.x);
  Serial.print(", ");
  Serial.print(currentPos.y);
  Serial.print(")  Heading: ");
  Serial.println(currentHeading * (180.0 / PI)); // Convert radians to degrees
}



// Find the next lookahead point
Point findLookaheadPoint() {
  for (int i = 0; i < (sizeof(desiredPositions) / sizeof(desiredPositions[0])) - 1; i++) {
    double dx = desiredPositions[i + 1].x - currentPos.x;
    double dy = desiredPositions[i + 1].y - currentPos.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance >= lookaheadDistance) {
      Serial.print("Lookahead Point: (");
      Serial.print(desiredPositions[i + 1].x);
      Serial.print(", ");
      Serial.print(desiredPositions[i + 1].y);
      Serial.print(")  ");
      return desiredPositions[i + 1];
    }
  }

  return desiredPositions[(sizeof(desiredPositions) / sizeof(desiredPositions[0])) - 1];
}

// Calculate curvature based on lookahead point
double calculateCurvature(Point lookaheadPoint) {
  double dx = lookaheadPoint.x - currentPos.x;
  double dy = lookaheadPoint.y - currentPos.y;
  double angleToLookahead = atan2(dy, dx);
  double alpha = angleToLookahead - currentHeading;

  double distance = sqrt(dx * dx + dy * dy);
  double curvature = (2 * sin(alpha)) / distance;

  // Log curvature
  Serial.print("Curvature: ");
  Serial.print(curvature);
  Serial.print("     ");

  return curvature;
}

// Adjust motor speeds based on curvature
void adjustMotorSpeeds(double curvature) {
  double leftMSpeed = baseSpeed * (1 - curvature);
  double rightMSpeed = baseSpeed * (1 + curvature);

  leftSpeed = constrain(leftMSpeed, 0, 16000);
  rightSpeed = constrain(rightMSpeed, 0, 16000);

  // Log motor speeds
  Serial.print("Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print("     ");

  Serial.print("Right Speed: ");
  Serial.print(rightSpeed);
  Serial.print("     ");
}

// Check if the path is complete
bool isPathComplete(Point targetPos, double threshold) {
  double dx = targetPos.x - currentPos.x;
  double dy = targetPos.y - currentPos.y;
  bool complete = sqrt(dx * dx + dy * dy) <= threshold;

  if (complete) {
    Serial.println("Reached target position.");
  }

  return complete;
}
