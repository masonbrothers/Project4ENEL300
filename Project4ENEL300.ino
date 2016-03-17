#include <Servo.h>

#define LEFT_MOTOR_PIN 11
#define RIGHT_MOTOR_PIN 10
Servo servoLeft;
Servo servoRight;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void stopMotors()
{
  servoLeft.attach(LEFT_MOTOR_PIN);
  servoRight.attach(10);
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}
