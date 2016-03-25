#include <Servo.h>

#define LEFT_SERVO_PIN 11
#define RIGHT_SERVO_PIN 10
#define FRONT_LED_PIN 5
#define FRONT_RECEIVER_PIN 13
#define RIGHT_LED_PIN 3
#define RIGHT_RECEIVER_PIN 4
#define BUZZER_PIN 9
#define WHISKER_PIN 8
#define WHISKER_LEFT_PIN 8
#define WHISKER_RIGHT_PIN 2
#define VISIBLE_LED_PIN 12

#define RIGHT_DIME_TURNING_TIME 610   // CALIBRATION TEST 1
#define LEFT_DIME_TURNING_TIME 630    // CALIBRATION TEST 2


#define RIGHT_PIVOT_TURNING_TIME 1260 // CALIBRATION TEST 3
#define LEFT_PIVOT_TURNING_TIME 1240  // CALIBRATION TEST 4
#define BOX_SIZE 2000
#define IR_DELAY_TIME 10

//#define TESTING

Servo servoLeft;
Servo servoRight;

double allignment = 0.89;  //Higher values make left motor stronger (makes it veer right)
                           //Lower values make right motor stronger (makes it veer left)

void calibration()
{

  int i;
  for (i = 0; i < 4; i++) // CALIBRATION TEST 1
  {
    turnPivotRight();
    delay(1000);
  }
  delay(3000);
  for (i = 0; i < 4; i++) // CALIBRATION TEST 2
  {
    turnPivotLeft();
    delay(1000);
  }
  /*
  for (i = 0; i < 4; i++) // CALIBRATION TEST 3
  {
    turnPivotRight();
    delay(1000);
  }
  delay(3000);
  for (i = 0; i < 4; i++) // CALIBRATION TEST 4
  {
    turnPivotLeft();
    delay(1000);
  }
  */
  /*
  beepTwoTimes();
  delay(3000);
  beepFiveTimes();
  */

}

void setup() {
  // put your setup code here, to run once:
  //STARTUP

  Serial.begin(9600);      // Initilizes serial out. This is used for debugging.
  int deltaLeaveStartZoneTime, deltaTimeToFirstCorner, deltaFindCupTime, deltaBoardLengthTime;
  int startingTime, roundedFirstCornerTime, hitBoardTime, startMeasuringWallTime, roundedThirdCornerTime, lastMeasuringWallTime, lastStretchTime;
  attachMotors();
  pinMode(VISIBLE_LED_PIN, OUTPUT);
  delay(1000);             // Gives a bit of time for us to position the robot before it starts moving
 
  #ifdef TESTING           // If we are doing unit tests, do not play the mission code.

  /*
  startServosForward();
  delay(1000);
  while (!irFrontSensorDetect());
  stopServos();
  delay(500);
  
  //Cup has been detected
  avoidObstacle();
  */
  //calibration();
  tryToHitTheBoard();
  while (1)
  {
    Serial.print("RightIR: ");
    Serial.print(irRightSensorDetect());
    Serial.print("\tFrontIR: ");
    Serial.print(irFrontSensorDetect());
    Serial.print("\tLeftWh: ");
    Serial.print(whiskerLeftSensorDetect());
    Serial.print("\tRightWh: ");
    Serial.print(whiskerRightSensorDetect());
    Serial.print("\n");
    if (irRightSensorDetect())
    {
      digitalWrite(VISIBLE_LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(VISIBLE_LED_PIN, LOW);
    }
  }
  #endif
  
  #ifndef TESTING
  
  startingTime = millis();          //Time at the begining of the test
  startServosForward();
  while (!whiskerFrontSensorDetect());    //Continue until whisker is triggered
  deltaLeaveStartZoneTime = getTimeSince(startingTime); //Get the change in time between starting to move and touching the wall 
  stopServos();
  delay(500);
  startServosBackward(); // Prevents the robot from hitting the board.
  delay(800);
  stopServos();
  delay(1000);
  turnPivotLeft();        // Turns left fast
  Serial.println("Going Forward");
  startServosForward();
  hitBoardTime = millis();
  while (irRightSensorDetect())
  {
    delay(IR_DELAY_TIME);
  }  // While board is there keep going forward
  //digitalWrite(VISIBLE_LED_PIN, LOW);
  //Turns around board
  
  deltaTimeToFirstCorner = getTimeSince(hitBoardTime);
  alignHitting();
  
  

  turnCorner();
  roundedFirstCornerTime = millis();
  
  startServosForward();
  while (!irFrontSensorDetect())
  {
    delay(IR_DELAY_TIME);
  }
  deltaFindCupTime = getTimeSince(roundedFirstCornerTime);
  stopServos();
  delay(500);
  
  //Cup has been detected
  avoidObstacle();     // Avoids obstacle and sets rover parallel to board.
  startServosForward(); 
  while (irRightSensorDetect())
  {
    delay(IR_DELAY_TIME);
  }  // While board is there keep going forward      // While board is there keep going forward
  
  
  alignHitting();
  
  turnLongCorner();
  turnPivotRight();
  tryToHitTheBoard(); // MASON FLAG
  startServosForward();
  startMeasuringWallTime = millis();
  while (irRightSensorDetect())
  {
    delay(IR_DELAY_TIME);
  }  // While board is there keep going forward
  
  deltaBoardLengthTime = getTimeSince(startMeasuringWallTime);
  alignHitting();
  
  turnCorner();
  roundedThirdCornerTime = millis();
  bool cupFound = true;
  while(!irFrontSensorDetect())
  {
    if (deltaFindCupTime + 1000 < getTimeSince(roundedThirdCornerTime)) // the cup is not there
    {
      stopServos();
      beepFiveTimes();
      cupFound = false;
      break;
    }
    delay(IR_DELAY_TIME);
    
  }
  if (cupFound)
  {
    stopServos();
    beepTwoTimes();
    avoidObstacle();
    startServosForward(); 
    while (irRightSensorDetect())
    {
      delay(IR_DELAY_TIME);
    }  // While board is there keep going forward
    
    alignHitting();
    
    turnLongCorner();
    
    turnPivotRight();
    tryToHitTheBoard(); // MASON FLAG
    startServosForward(); 
    lastMeasuringWallTime = millis();
    while(1)
    {
      if (deltaBoardLengthTime - deltaTimeToFirstCorner - 750 < getTimeSince(lastMeasuringWallTime))
        break;
    }
    stopServos();
    turnPivotLeft();
    startServosForward();
    lastStretchTime = millis();
    /*
    //This code uses the timer to find home
    while(1)
    {
      if (deltaLeaveStartZoneTime < getTimeSince(lastStretchTime))
        break;
    }
    stopServos();
    */
    //This code uses the front sensors to determine when it has hit the backdrop and is home.
    while (!whiskerFrontSensorDetect());    //Continue until whisker is triggered
    stopServos();
  }
  //START
  #endif
  
}

void alignHitting()
{
  // TEST HITTING
  stopServos();
  delay(500);
  startServosBackward();
  delay(1300);
  turnPivotRight();
  stopServos();
  delay(500);
  startServosBackward();
  delay(800);
  tryToHitTheBoard(); // MASON FLAG
}

void turnCorner()
{
  startServosForward();
  delay(1000); //Continue Driving forward a bit
  turnPivotRight();
  startServosForward();
  delay(1500); //Get to the other side of the board
  turnPivotRight();    
  startServosForward();
  delay(1500); // Drive forward so that the sensor will see the board.
}

void turnLongCorner()
{
  startServosForward();
  delay(1000); //Continue Driving forward a bit
  turnPivotRight();
  startServosForward();
  delay(3000); //Get to the other side of the board
  turnPivotRight();    
  startServosForward();
  delay(1500); // Drive forward so that the sensor will see the board.
}

void avoidObstacle()
{
  startServosForward();
  delay(200);
  turnPivotLeft();
  startServosForward();
  delay(BOX_SIZE/2);
  turnPivotRight();
  startServosForward();
  delay(BOX_SIZE);
  turnPivotRight();
  startServosForward();
  delay(BOX_SIZE*2/5);
  tryToHitTheBoard();
}

void tryToHitTheBoard()
{
  setServosNoFactor(20,20);
  while ((!whiskerLeftSensorDetect())&&(!whiskerRightSensorDetect()));
  stopServos();
  while(1)
  {
    bool left = whiskerLeftSensorDetect();
    bool right = whiskerRightSensorDetect();
    if((!whiskerLeftSensorDetect())&&(!whiskerRightSensorDetect()))
    {
      delay(100);
      if (whiskerLeftSensorDetect()&&whiskerRightSensorDetect())
        break;
    }
    if (left&&right)
      break;
    else if (left)
    {
      setServosNoFactor(-20,-20);
      delay(300);
      setServosNoFactor(-20,20);
      delay(200);
    }
    else if (right)
    {
      setServosNoFactor(-20,-20);
      delay(300);
      setServosNoFactor(20,-20);
      delay(200);

    }
    setServosNoFactor(20,20);
    delay(300);
    
    
  }
  stopServos();
  delay(500);
  startServosBackward(); // Prevents the robot from hitting the board.
  delay(1000);
  stopServos();
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
  delay(LEFT_DIME_TURNING_TIME);
  stopServos();
}

void turnPivotLeft()
{
  startPivotTurningLeft();
  delay(LEFT_PIVOT_TURNING_TIME);
  stopServos();
}

void startPivotTurningLeft()
{
  setServos(0, 100);
}

void turnDimeRight()
{
  startDimeTurningRight();
  delay(RIGHT_DIME_TURNING_TIME);
  stopServos();
}
void startDimeTurningLeft()
{
  setServos(-100, 100);
}

void turnPivotRight()
{
  startPivotTurningRight();
  delay(RIGHT_PIVOT_TURNING_TIME);
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

void setServosNoFactor(int left, int right) //Convention is Positive numbers for forward
{
  servoLeft.writeMicroseconds(1500+(double)left); // Positive for forward, negative for backward
  servoRight.writeMicroseconds(1500-(double)right); // Negative for forward, positive for backward
}


void beepTwoTimes()
{
  int i;
  for(i = 0; i < 2; i++)
  {
    tone(BUZZER_PIN, 1000, 500);
    delay(1000);
    noTone(BUZZER_PIN);
  }
}

void beepFiveTimes()
{
  int i;
  for(i = 0; i < 5; i++)
  {
    tone(BUZZER_PIN, 1000, 500);
    delay(1000);
    noTone(BUZZER_PIN);
  }
}

boolean whiskerFrontSensorDetect()
{
  return (whiskerLeftSensorDetect() || whiskerRightSensorDetect()); //When the pin goes low, the whisker has touched.
}

boolean whiskerLeftSensorDetect()
{
  return !digitalRead(WHISKER_LEFT_PIN); //When the pin goes low, the whisker has touched.
}

boolean whiskerRightSensorDetect()
{
  return !digitalRead(WHISKER_RIGHT_PIN); //When the pin goes low, the whisker has touched.
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
  for (int i = 0; i < 20; i++)
  {
    count += irDetect(irLedPin, irReceiverPin, 38000);
    delay(1);
  } 
  if (count > 15)
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
  return irFrontSensorDetect();
  setServos(0,0);
}
