#include <AFMotor.h>
#include <Servo.h>
#include <Arduino.h>
#include "logger.h" // Include your logging library

// Motor definitions
AF_DCMotor motor1(1); // Motor 1
AF_DCMotor motor2(2); // Motor 2
AF_DCMotor motor3(3); // Motor 3
AF_DCMotor motor4(4); // Motor 4

// Servo definition
Servo ultraServo;

const int trigPin = 8;
const int echoPin = 7;
const int ultraServoPin = 10;

// Obstacle avoidance constants
const int obstacleThreshold = 40; // cm
const int maxMeasurements = 19;
const int measureInterval = 10;
int motorSpeed = 20; // Base speed of motors, slooooowwww

void setup()
{
  Serial.begin(115200);
  // Initialize motors
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);
  motor3.setSpeed(motorSpeed);
  motor4.setSpeed(motorSpeed);

  // Initialize Servo
  ultraServo.attach(ultraServoPin);
  ultraServo.write(90); // Center position

  // Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize logging
  Logger::log(INFO, "System initialized.");
}

int getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (round trip)

  Logger::log(INFO, "Distance measured: " + String(distance) + " cm");
  return distance;
}

void speedAdjust(int speedAdjust)
{
  motorSpeed += speedAdjust;
  Logger::log(INFO, "Adjusting speed with: " + String(speedAdjust));
}

void leftMotorsAdjust(int speedAdjust)
{
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);
  motor3.setSpeed(motorSpeed + speedAdjust);
  motor4.setSpeed(motorSpeed + speedAdjust);

  Logger::log(INFO, "Adjusting left motors with speed: " + String(speedAdjust));
}

void rightMotorsAdjust(int speedAdjust)
{
  motor1.setSpeed(motorSpeed + speedAdjust);
  motor2.setSpeed(motorSpeed + speedAdjust);
  motor3.setSpeed(motorSpeed);
  motor4.setSpeed(motorSpeed);

  Logger::log(INFO, "Adjusting right motors with speed: " + String(speedAdjust));
}

void moveForward()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  Logger::log(INFO, "Moving forward.");
}

void stopMotors()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  Logger::log(INFO, "Motors stopped.");
}

/**
 * Turn left with a speed adjustment
 * @param speedAdjust The speed at which the inner motors speed is slowed down
 */
void turnLeft(int speedAdjust)
{
  motor1.setSpeed(motorSpeed - speedAdjust);
  motor2.setSpeed(motorSpeed - speedAdjust);
  motor3.setSpeed(motorSpeed + speedAdjust);
  motor4.setSpeed(motorSpeed + speedAdjust);

  Logger::log(INFO, "Turning left with adjustment: " + String(speedAdjust));
}

/**
 * Turn right with a speed adjustment
 * @param speedAdjust The speed at which the inner motors speed is slowed down
 */
void turnRight(int speedAdjust)
{
  rightMotorsAdjust(speedAdjust);

  Logger::log(INFO, "Turning right with adjustment: " + String(speedAdjust));
}

/**
 * Main loop
 *
 * The robot will move forward until an obstacle is detected.
 * When an obstacle is detected, the robot will turn towards the direction with the most distance.
 * If no direction has a distance greater than the threshold, the robot will choose a random direction.
 *
 * The robot will then move forward again.
 */
void loop()
{
  int distances[maxMeasurements];
  int maxDistance = 0;
  int maxIndex = 0;

  for (int i = 0; i < maxMeasurements; i++)
  {
    int angle = map(i, 0, maxMeasurements - 1, 0, 180);
    ultraServo.write(angle);
    delay(100); // Allow the servo to reach the position
    int distance = getDistance();
    distances[i] = distance;

    if (distance > maxDistance)
    {
      maxDistance = distance;
      maxIndex = i;
    }

    if (i % measureInterval == 0 && maxDistance > obstacleThreshold)
    {
      // Turn towards the direction of the maximum distance
      if (maxIndex < maxMeasurements / 2)
      {
        turnLeft(map(maxIndex, 0, maxMeasurements / 2, 0, motorSpeed));
      }
      else
      {
        turnRight(map(maxIndex, maxMeasurements / 2, maxMeasurements - 1, 0, motorSpeed));
      }
      break;
    }
  }

  if (maxDistance <= obstacleThreshold)
  {
    stopMotors();
    Logger::log(WARNING, "Obstacle detected. Choosing a new direction.");

    delay(500);
  }
  else
  {
    moveForward();
  }
}
