#include <Servo.h>

#define LEFT_SERVO_PIN 11
#define RIGHT_SERVO_PIN 10
Servo servoLeft;
Servo servoRight;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void attachMotors()
{
  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);
}

void stopServos()
{
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}

void startServosForward()
{
  setServos(100, 100);
}

void startServosBackward()
{
  setServos(-100, 100);
}

int getTimeSince(int input) //In milliseconds
{
  return millis() - input;
}

boolean detectedInFront()
{
  
}

boolean detectedOnRight()
{
  
}

void turnLeft()
{
  startTurningLeft();
  delay(1000);
  stopServos();
}

void turnRight()
{
  startTurningRight();
  delay(1000);
  stopServos();
}
void startTurningLeft()
{
  setServos(-100, 100);
}

void startTurningRight()
{
  setServos(100, -100);
}

/*
  setServos allows one to use positive offsets from 1500 (stop). 
  The convention is that positive values will move the robot forward and
  negative values will move the robot backwards.
  */
void setServos(int left, int right) //Convention is Positive numbers for forward
{
  servoLeft.writeMicroseconds(1500+left); // Positive for forward, negative for backward
  servoRight.writeMicroseconds(1500-right); // Negative for forward, positive for backward
}
