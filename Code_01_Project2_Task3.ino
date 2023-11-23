#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

#define DISTANCE_THRESHOLD 20 // Set the distance threshold for obstacle detection in centimeters
#define TARGET_DISTANCE 312 // The target distance from start to end in centimeters
#define MAX_DIST_TIME 10000 // Maximum time to reach the target distance in milliseconds (10 seconds)
#define PID_KP 1.0 // Proportional gain
#define PID_KI 0.1 // Integral gain
#define PID_KD 0.01 // Derivative gain
#define PIN_Motor_PWMA 5 // PWM pin for right side motors
#define PIN_Motor_PWMB 6 // PWM pin for left side motors
#define PIN_Motor_BIN_1 8 // Voltage pin for left side motors
#define PIN_Motor_AIN_1 7 // Voltage pin for right side motors
#define PIN_Motor_STBY 3 // Voltage pin for both motors
#define TRIG_PIN 13 // Trigger pin for the ultrasonic sensor
#define ECHO_PIN 12 // Echo pin for the ultrasonic sensor

float previousError = 0;
float integral = 0;

void setup() {
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
}

void loop() {
  unsigned long startTime = millis();
  unsigned long elapsedTime;
  float initialDistance = getTotalDistance(startTime);
  // Move forward until the target distance is reached or the time limit is reached
  while (true) {
    elapsedTime = millis() - startTime;

    if (elapsedTime >= MAX_DIST_TIME) {
      // Time limit reached, stop
      stopRobot();
      break;
    }

    // Calculate PID correction
    float gyroValue = mpu6050.getAngleZ();
    float correction = calculatePIDCorrection(gyroValue);

    // Apply correction to motor speeds
    int speed1 = 140 + correction;
    int speed2 = 145 - correction;
    forward(speed1, speed2);
    delay(10); // Delay for stability and to reduce loop frequency
    
    float currentDistance = getTotalDistance(startTime);

    if (currentDistance - initialDistance >= TARGET_DISTANCE) {
      // Target distance reached, stop
      stopRobot();
      break;
    }
    if (isObstacleDetected()) {
      // Obstacle detected, stop and take a fixed diversion
      stopRobot();
      delay(500); // Pause for 0.5 seconds

      // Divert around the obstacle
      divertObstacle();

      // Return to the original path
      returnToPath();

      // Update start time to include the time spent on diversion
      startTime = millis();
    }
  }
}

float calculatePIDCorrection(float gyroValue) {
  float error = gyroValue; // Error is the deviation from the desired angle (assumed 0 for straight)
  integral += error;
  float derivative = error - previousError;

  float correction = PID_KP * error + PID_KI * integral + PID_KD * derivative;

  previousError = error;

  return correction;
}

float getTotalDistance(unsigned long startTime) {
  float elapsedTime = (millis() - startTime) / 1000.0; // Convert milliseconds to seconds
  float speed = 0.5; // Adjust the speed based on your robot's behavior
  return speed * elapsedTime;
}

bool isObstacleDetected() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;

  return (distance < DISTANCE_THRESHOLD);
}

void divertObstacle() {
  // For simplicity, we'll turn left for 90 degrees
  left90();
  delay(500); // Pause for 0.5 seconds to allow the robot to complete the turn

  // Move forward to clear the obstacle
  forward(140, 140);
  delay(300); 

  // Turn back in the opposite direction to clear length of obstacle
  right90();
  delay(500); // Pause for 0.5 seconds to allow the robot to complete the turn

  // Move forward to clear the obstacle
  forward(140, 140);
  delay(500); 

   // Turn back in the opposite direction to return to the original path
  right90();
  delay(500); // Pause for 0.5 seconds to allow the robot to complete the turn

  // Move forward to clear the obstacle
  forward(140, 140);
  delay(400); 
  stopRobot(); // Stop the robot
   // Delay for stability
}

void returnToPath() {
  left90();
  delay(500); // Pause for 0.5 seconds to allow the robot to complete the turn
}

// Your existing motor control functions (moveForward, stopRobot, turnLeft90, turnRight90)
void forward(int speed1, int speed2)//function for robot to move forward at a speed input by the user
{
  digitalWrite(PIN_Motor_STBY, HIGH); //allow high voltage control for both motors
  digitalWrite(PIN_Motor_AIN_1, HIGH); //Set high voltage for right side motors
  digitalWrite(PIN_Motor_BIN_1, HIGH); //Set high voltage for left side motors
  analogWrite(PIN_Motor_PWMA, speed1); //Set speed for right side motors
  analogWrite(PIN_Motor_PWMB, speed2); //Set speed for left side motors
}
void backward(int speed)//function for robot to move backwards at a speed input by the user
{
  digitalWrite(PIN_Motor_STBY, HIGH); //allow high voltage control for both motors
  digitalWrite(PIN_Motor_AIN_1, LOW); //Set low voltage for right side motors
  digitalWrite(PIN_Motor_BIN_1, LOW); //Set low voltage for left side motors
  analogWrite(PIN_Motor_PWMA, speed); //Set speed for right side motors
  analogWrite(PIN_Motor_PWMB, speed); //Set speed for left side motors
}

void left90(void) //turn left function
{
  digitalWrite(PIN_Motor_STBY, HIGH); //allow high voltage control for both motors
  digitalWrite(PIN_Motor_AIN_1, HIGH); //Set high voltage for right side motors
  digitalWrite(PIN_Motor_BIN_1, LOW); //Set low voltage for left side motors
  analogWrite(PIN_Motor_PWMA, 255); //Set speed for right side motors
  analogWrite(PIN_Motor_PWMB, 255); //Set speed for left side motors
  delay(145);//chaning this delay, changes how much the robot turns (larger delay, larger turn angle)
  digitalWrite(PIN_Motor_STBY, LOW); //Stop both motors
}


void right90(void) //turn right function
{
  digitalWrite(PIN_Motor_STBY, HIGH); //allow high voltage control for both motors
  digitalWrite(PIN_Motor_AIN_1, LOW); //Set high voltage for right side motors
  digitalWrite(PIN_Motor_BIN_1, HIGH); //Set low voltage for left side motors
  analogWrite(PIN_Motor_PWMA, 255); //Set speed for right side motors
  analogWrite(PIN_Motor_PWMB, 255); //Set speed for left side motors
  delay(160);//chaning this delay, changes how much the robot turns (larger delay, larger turn angle)
  digitalWrite(PIN_Motor_STBY, LOW); //Stop both motors
}

void stopRobot() {
  digitalWrite(PIN_Motor_STBY, LOW); 
  delay(100000);// Stop both motors
}
