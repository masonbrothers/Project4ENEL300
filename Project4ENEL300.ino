#include <Servo.h>

#define LEFT_SERVO_PIN 11
#define RIGHT_SERVO_PIN 10
#define FRONT_LED_PIN 5
#define FRONT_RECEIVER_PIN 13
#define RIGHT_LED_PIN 3
#define RIGHT_RECEIVER_PIN 4
#define BUZZER_PIN 2
#define WHISKER_PIN 8
#define TESTING

Servo servoLeft;
Servo servoRight;

double allignment = 0.89;  //Higher values make left motor stronger (makes it veer right)
                           //Lower values make right motor stronger (makes it veer left)

void setup() {
  // put your setup code here, to run once:
  //STARTUP
  Serial.begin(9600);
  int leaveStartZoneTime, edgeOneTime, findCupTime, boardLengthTime;
  int totalStartingTime;
  attachMotors();
  delay(1000);
  #ifdef TESTING
  int i;
  for (i = 0; i < 4; i++)
  {
    turnPivotRight();
    delay(500);
  }
  for (i = 0; i < 4; i++)
  {
    turnPivotLeft();
    delay(500);
  }
  
  /*
  for (i = 0; i < 4; i++)
  {
    turnDimeRight();
    delay(500);
  }
  */
  /*
  for (i = 0; i < 4; i++)
  {
    turnDimeLeft();
    delay(500);
  }
  */
  /*
  startServosForward();
  delay(1000);
  while (!irFrontSensorDetect());
  stopServos();
  delay(500);
  
  //Cup has been detected
  avoidObstacle();
  */
  while (1)
  {
    Serial.print("Right: ");
    Serial.print(irRightSensorDetect());
    Serial.print("\tFront: ");
    Serial.print(irFrontSensorDetect());
    Serial.print("\tWhisker: ");
    Serial.print(whiskerFrontSensorDetect());
    Serial.print("\n");
  }
  #endif
  
  #ifndef TESTING
  totalStartingTime = millis();
  startServosForward();
  while (!whiskerFrontSensorDetect());
  leaveStartZoneTime = getTimeSince(totalStartingTime);
  stopServos();
  delay(500);
  startServosBackward(); //Prevents the robot from hitting the board.
  delay(500);
  stopServos();
  delay(1000);
  turnDimeLeft();
  startServosForward();
  while (irRightSensorDetect());
  delay(1000); //Continue Driving forward a bit
  turnDimeRight();
  startServosForward();
  delay(2000); //Get to the other side of the 
  turnDimeRight();
  startServosForward();
  delay(1000);
  
  while (!irFrontSensorDetect());
  stopServos();
  delay(500);
  
  //Cup has been detected
  avoidObstacle();
  //START
  #endif
  
}

void avoidObstacle()
{
  int boxSize = 2000;
  startServosBackward();
  delay(boxSize/2);
  turnPivotLeft();
  startServosForward();
  delay(boxSize);
  turnPivotRight();
  startServosForward();
  delay(boxSize);
  turnPivotRight();
  startServosForward();
  delay(boxSize);
  turnPivotLeft();
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
  setServos(0, 0);
}

void startServosForward()
{
  setServos(100, 100);
}

void startServosBackward()
{
  setServos(-100, -100);
}

int getTimeSince(int input) //In milliseconds
{
  return millis() - input;
}

void turnDimeLeft()
{
  startDimeTurningLeft();
  delay(620);
  stopServos();
}

void turnPivotLeft()
{
  startPivotTurningLeft();
  delay(1275);
  stopServos();
}

void startPivotTurningLeft()
{
  setServos(0, 100);
}

void turnDimeRight()
{
  startDimeTurningRight();
  delay(650);
  stopServos();
}
void startDimeTurningLeft()
{
  setServos(-100, 100);
}

void turnPivotRight()
{
  startPivotTurningRight();
  delay(1275);
  stopServos();
}

void startPivotTurningRight()
{
  setServos(100, 0);
}


void startDimeTurningRight()
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
  servoLeft.writeMicroseconds(1500+(double)left*allignment); // Positive for forward, negative for backward
  servoRight.writeMicroseconds(1500-(double)right/allignment); // Negative for forward, positive for backward
}

void beepTwoTimes()
{
  int i;
  for(i = 0; i < 2; i++)
  {
    tone(BUZZER_PIN, 1000, 500);
    delay(500);
  }
}

void beepFiveTimes()
{
  int i;
  for(i = 0; i < 5; i++)
  {
    tone(BUZZER_PIN, 1000, 500);
    delay(500);
  }
}

boolean whiskerFrontSensorDetect()
{
  return !digitalRead(WHISKER_PIN); //When the pin goes low, the whisker has touched.
}

boolean irFrontSensorDetect()
{
  return irSensorDetect(FRONT_LED_PIN, FRONT_RECEIVER_PIN);
}

boolean irRightSensorDetect()
{
  return irSensorDetect(RIGHT_LED_PIN, RIGHT_RECEIVER_PIN);
}

boolean irSensorDetect(int irLedPin, int irReceiverPin)
{
  int count = 0;
  for (int i = 0; i < 10; i++)
  {
    count += irDetect(irLedPin, irReceiverPin, 38000);
  } 
  if (count > 8)
    return true;
  return false;
}


int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = !digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}  

void zigZagMason()
{
  while(1)
  {
    if (interrupt()) break;
    if (irRightSensorDetect())
    {
      setServos(25, 50);
    }
    else
    {
      setServos(50,25);
    }
    delay(10);
  }
}

bool interrupt()
{
  irRightSensorDetect();
  setServos(0,0);
}
