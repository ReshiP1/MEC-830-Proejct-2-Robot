#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

#define DISTANCE_THRESHOLD 20 // Set the distance threshold for obstacle detection in centimeters
#define TARGET_DISTANCE 156 // The target distance from point A to B in centimeters
#define MAX_DIST_TIME 8380 // Maximum time to reach the target distance in milliseconds (8.38 seconds)
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
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  moveDistance(TARGET_DISTANCE); // Move forward for the target distance
  right90(); // Turn right
  moveDistance(TARGET_DISTANCE); // Move forward for the target distance
  left90(); // Turn left
  moveDistance(TARGET_DISTANCE); // Move forward for the target distance
  stopRobot(); // Stop the robot
}

void moveDistance(int targetDistance) {
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  float currentDistance = 0;

  while (currentDistance < targetDistance && elapsedTime < MAX_DIST_TIME) {
    // Calculate PID correction
    float gyroValue = mpu6050.getAngleZ();
    float correction = calculatePIDCorrection(gyroValue);

    // Apply correction to motor speeds
    int speed1 = 56 + correction;
    int speed2 = 60 - correction;
    forward(speed1, speed2);

    delay(10); // Delay for stability and to reduce loop frequency

    // Calculate the distance using the gyro
    currentDistance += calculateDistance(gyroValue);

    elapsedTime = millis() - startTime;
  }

  stopRobot(); // Stop the robot
}

float calculatePIDCorrection(float gyroValue) {
  float error = gyroValue; // Error is the deviation from the desired angle (assumed 0 for straight)
  integral += error;
  float derivative = error - previousError;

  float correction = PID_KP * error + PID_KI * integral + PID_KD * derivative;

  previousError = error;

  return correction;
}

float calculateDistance(float gyroValue) {
  // Assuming a linear relationship between gyro values and distance
  float slope = 0.02; 
  float offset = 0.0; 
  return gyroValue * slope + offset;
}

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
  analogWrite(PIN_Motor_PWMA, 235); //Set speed for right side motors
  analogWrite(PIN_Motor_PWMB, 235); //Set speed for left side motors
  delay(188);//chaning this delay, changes how much the robot turns (larger delay, larger turn angle)
  digitalWrite(PIN_Motor_STBY, LOW); //Stop both motors
}


void right90(void) //turn right function
{
  digitalWrite(PIN_Motor_STBY, HIGH); //allow high voltage control for both motors
  digitalWrite(PIN_Motor_AIN_1, LOW); //Set high voltage for right side motors
  digitalWrite(PIN_Motor_BIN_1, HIGH); //Set low voltage for left side motors
  analogWrite(PIN_Motor_PWMA, 235); //Set speed for right side motors
  analogWrite(PIN_Motor_PWMB, 235); //Set speed for left side motors
  delay(185);//chaning this delay, changes how much the robot turns (larger delay, larger turn angle)
  digitalWrite(PIN_Motor_STBY, LOW); //Stop both motors
}


void stopRobot() {
  digitalWrite(PIN_Motor_STBY, LOW); // Stop both motors
  delay(100000); // Delay for stability
}
