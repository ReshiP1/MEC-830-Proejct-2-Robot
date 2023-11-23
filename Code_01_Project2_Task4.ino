#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

#define PIN_Motor_PWMA 5  // PWM RIGHT
#define PIN_Motor_PWMB 6  // PWM LEFT

#define PIN_Motor_STBY 3  // VOLTAGE BOTH MOTORS
#define PIN_Motor_AIN_1 7 // VOLTAGE RIGHT
#define PIN_Motor_BIN_1 8 // VOLTAGE LEFT

#define TARGET_DISTANCE 156 // Target distance for each side of the square in centimeters

float previousError = 0;
float integral = 0;
float gyroCorrectionFactor = 1.0; // Adjust this value based on your robot's behavior
float straightAngle = 0.0; // Target angle for keeping the car straight

void setup()
{
  Serial.begin(9600); // Start serial monitor
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

void loop()
{
  for (int i = 0; i < 4; ++i)
  {
    travelDistance(56, 60, TARGET_DISTANCE);
    turn90UsingGyro(1); // 1 for right turn
  }
}

void travelDistance(int speed1, int speed2, float targetDistance)
{
  unsigned long startTime = millis();
  unsigned long elapsedTime;

  while (true)
  {
    elapsedTime = millis() - startTime;

    if (elapsedTime > 5000) // Allow some time for the travel
    {
      stop();
      break;
    }

    float gyroValue = mpu6050.getAngleZ();
    float correction = calculatePIDCorrection(gyroValue, straightAngle);

    int correctedSpeed1 = speed1 + correction;
    int correctedSpeed2 = speed2 - correction;
    forward(correctedSpeed1, correctedSpeed2);
    delay(10); // Delay for stability and to reduce loop frequency
  }
}

void forward(int speed1, int speed2)
{
  digitalWrite(PIN_Motor_STBY, HIGH);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, speed1);
  analogWrite(PIN_Motor_PWMB, speed2);
}

void stop(void)
{
  digitalWrite(PIN_Motor_STBY, LOW);
  delay(1000); // Stop both motors and wait for stability
}

void turn90UsingGyro(int direction)
{
  unsigned long startTime = millis();
  unsigned long elapsedTime;
  int turnSpeed = 235;

  while (true)
  {
    elapsedTime = millis() - startTime;

    if (elapsedTime > 500) // Allow some time for the turn
    {
      stop();
      break;
    }

    float gyroValue = mpu6050.getAngleZ();
    float targetAngle = direction * 90.0; // 90 degrees for a right turn, -90 for a left turn
    float correction = calculatePIDCorrection(gyroValue, targetAngle);

    int speed1 = turnSpeed - direction * correction;
    int speed2 = turnSpeed + direction * correction;
    forward(speed1, speed2);
    delay(10); // Delay for stability and to reduce loop frequency
  }
}

float calculatePIDCorrection(float gyroValue, float targetAngle)
{
  float error = targetAngle - gyroValue;
  integral += error;
  float derivative = error - previousError;

  float correction = gyroCorrectionFactor * (error + 0.1 * integral + 0.01 * derivative);
  previousError = error;

  return correction;
}
