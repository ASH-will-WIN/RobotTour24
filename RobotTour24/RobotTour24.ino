#include <Math.h>
#include <AccelStepper.h>


// Left is pin1 side from Northview code
// Right is pin2 side from Northview code
#define DIR_PIN_LEFT         2      
#define STEP_PIN_LEFT         3      

#define DIR_PIN_RIGHT         4
#define STEP_PIN_RIGHT        5


// For 1/16th microstepping
#define MS1 6
#define MS2 7
#define MS3 8




AccelStepper leftMotor(AccelStepper::DRIVER, STEP_PIN_LEFT, DIR_PIN_LEFT);
AccelStepper rightMotor(AccelStepper::DRIVER, STEP_PIN_RIGHT, DIR_PIN_RIGHT);

long previousTime = millis();

struct Point {
  double x;  // x-coordinate
  double y;  // y-coordinate
};

double lookaheadDistance = 10;
double baseSpeed = 500;
double maxSpeed = 2000;
double scaleFactor = 1;

double currentLeftSteps = 0;
double currentRightSteps = 0;
double previousLeftSteps = 0;
double previousRightSteps = 0;


double rightSpeed;
double leftSpeed;



double WHEELBASE = 15.875;
double WHEEL_RADIUS = 4.0132;
double WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
double STEPS_PER_REV = 200 * 16;  // 16 is the 1/16 microstepping
double dStep = WHEEL_CIRCUMFERENCE/STEPS_PER_REV;
double totalDistance = 0;


double initialHeading = 0;
double currentHeading = initialHeading;
Point initialPos = {0, 0};
Point currentPos = {0, 0};
Point desiredPositions[5] = {{0, 0},
                                 {0, 10},
                                 {0, 20},
                                 {0, 30},
                                 {0, 40}};





void setup() {
  // Initialize hardware serial for debugging
  Serial.begin(115200);


  // Set pin modes for both motors
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);

  pinMode(STEP_PIN_LEFT, OUTPUT);
  pinMode(STEP_PIN_RIGHT, OUTPUT);



  // Microstepping pins:
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  // Set microstepping to 1/16
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  rightMotor.setSpeed(50);
  leftMotor.setSpeed(50);
  rightMotor.setAcceleration(50);
  leftMotor.setAcceleration(50);
  rightMotor.moveTo(500);
  leftMotor.moveTo(500);
  rightMotor.run();
  leftMotor.run();
}


void loop() {

  updateValues();

  adjustMotorSpeeds(calculateCurvature(findLookaheadPoint()));

  leftMotor.setSpeed(constrain(leftSpeed, 0, maxSpeed));
  rightMotor.setSpeed(constrain(rightSpeed, 0, maxSpeed));


  if(isPathComplete(desiredPositions[4], 3)) {
    leftMotor.stop();
    rightMotor.stop();

    while(true){}
  }

  leftMotor.run();
  rightMotor.run();

  // Serial.print("LStepsTotal: "); Serial.print(leftMotor.currentPosition()); 
  // Serial.print("    RStepsTotal: "); Serial.print(rightMotor.currentPosition());
  // Serial.print("    Heading: "); Serial.print(currentHeading);
  // Serial.print("    XPos: "); Serial.print(currentPos.x); Serial.print("  YPos: "); Serial.println(currentPos.y);
  // Serial.println(" ");

  if (millis() - previousTime >= 100) {
        Serial.print("LStepsTotal: "); Serial.print(leftMotor.currentPosition());
        Serial.print("    RStepsTotal: "); Serial.print(rightMotor.currentPosition());
        Serial.print("    Heading: "); Serial.print(currentHeading);
        Serial.print("    XPos: "); Serial.print(currentPos.x); Serial.print("  YPos: "); Serial.println(currentPos.y);
        previousTime = millis();
  }
}



void updateHeading() {
  double deltaSteps = currentLeftSteps - currentRightSteps;
  double deltaHeading = (deltaSteps * dStep)/WHEELBASE;
  currentHeading += deltaHeading;

  currentHeading = fmod(currentHeading, 2 * PI);
  if (currentHeading < 0) {
    currentHeading += 2 * PI;
  }
}


void updateDistanceAndPos() {
  double avgDeltaSteps = ((currentLeftSteps - previousLeftSteps) + (currentRightSteps - previousRightSteps))/2;
  double deltaDistance = avgDeltaSteps * dStep;
  
  totalDistance += deltaDistance;
  currentPos.x += deltaDistance * cos(currentHeading);
  currentPos.y += deltaDistance * sin(currentHeading);
}

void updateStepCounts(){
  previousLeftSteps = currentLeftSteps;
  previousRightSteps = currentRightSteps;

  currentLeftSteps = leftMotor.currentPosition();
  currentRightSteps = rightMotor.currentPosition();
}

void updateValues(){
  updateStepCounts();
  updateHeading();
  updateDistanceAndPos();
}

Point findLookaheadPoint(){
  for (int i = 0; i < (sizeof(desiredPositions)/sizeof(desiredPositions[0])) - 1; i++){
    double dx = desiredPositions[i + 1].x - currentPos.x;
    double dy = desiredPositions[i + 1].y - currentPos.y;
    double distance = sqrt(dx*dx + dy*dy);

    if (distance >= lookaheadDistance) {
      return desiredPositions[i + 1];
    }
  }

  return desiredPositions[(sizeof(desiredPositions)/sizeof(desiredPositions[0])) - 1];
}


double calculateCurvature(Point lookaheadPoint) {
  double dx = lookaheadPoint.x - currentPos.x;
  double dy = lookaheadPoint.y - currentPos.y;
  double angleToLookahead = atan2(dy, dx);
  double alpha = angleToLookahead - currentHeading;

  double distance = sqrt(dx*dx + dy*dy);
  double curve = (2 * sin(alpha))/distance;
  return curve;
}


void adjustMotorSpeeds(double k) {
  double leftMSpeed = baseSpeed * (1 - k * WHEELBASE/2);
  double rightMSpeed = baseSpeed * (1 + k * WHEELBASE/2);
  
  leftSpeed = constrain(leftMSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightMSpeed, 0, maxSpeed);
}



bool isPathComplete(Point targetPos, double threshold) {
  double dx = targetPos.x - currentPos.x;
  double dy = targetPos.y - currentPos.y;
  double distance = sqrt(dx * dx + dy * dy);
  return distance <= threshold;
}


double millisToSeconds(double ms) {
  return ms/1000.0000;
}


double secondsToMillis(double s) {
  return s * 1000.0000;
}
