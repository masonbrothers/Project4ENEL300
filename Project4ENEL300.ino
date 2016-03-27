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

#define RIGHT_DIME_TURNING_TIME 610 
#define LEFT_DIME_TURNING_TIME 630   


#define RIGHT_PIVOT_TURNING_TIME 1310 // CALIBRATION TEST 1 @7.3v
#define LEFT_PIVOT_TURNING_TIME 1250  // CALIBRATION TEST 2 @7.3v
#define BOX_SIZE 2000
#define IR_DELAY_TIME 10

#define TESTING

Servo servoLeft;
Servo servoRight;

double allignment = 0.92; 
//straight alignment factor 0.89@7.3v, 0.95@7.28v
//
//Higher values make left motor stronger (makes it veer right)
//Lower values make right motor stronger (makes it veer left) 

void calibration()
{

  int i;
  /*
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
  */
  /*
  for (i = 0; i < 4; i++) // CALIBRATION TEST 3
  {
    turnPivotBackwardRight();
    delay(1000);
  }
  delay(3000);
  for (i = 0; i < 4; i++) // CALIBRATION TEST 4
  {
    turnPivotBackwardLeft();
    delay(1000);
  }
*/
  /*
  beepTwoTimes();
  delay(3000);
  beepFiveTimes();
  */
  
  startServosForward();
  delay(9001);
  stopServos();

}

void setup() {
  // put your setup code here, to run once:
  //STARTUP

  Serial.begin(9600);                                                          // Initilizes serial out. This is used for debugging.
  //Note: delta is used to denote changes in time. These are used to determine lengths.
  int deltaLeaveStartZoneTime; //start zone to board
  int deltaTimeToFirstCorner;  //initial contact with board to left end of board
  int deltaFindCupTime;        //time from left end to cup 1st time
  int deltaBoardLengthTime;    //time to traverse entire side of board (non cup side measured)
  
  int startingTime;           //global starting time of entire code
  int roundedFirstCornerTime; //time as soon as corner is rounded and robot is parallel to board 1st time
  int toEdgeOfBoardFirstTime;
  int startMeasuringWallTime;
  int roundedThirdCornerTime;
  int lastMeasuringWallTime;
  int lastStretchTime;
  attachMotors();
  pinMode(VISIBLE_LED_PIN, OUTPUT);
  delay(1000);             // Gives a bit of time for us to position the robot before it starts moving
 
  #ifdef TESTING           // If we are doing unit tests, do not play the mission code.

/*
    while (!whiskerFrontSensorDetect());    //Continue until whisker is triggered
    stopServos();
    startServosBackward();
    delay(200);
    stopServos();
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
  //calibration();
  tryToHitTheBoardBeta();
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
    if (irFrontSensorDetect())
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
  //starting zone to board
  startingTime = millis();                                                  //Time at the begining of the test
  startServosForward();                                                     //Starts going forward towards board
  while (!whiskerFrontSensorDetect());                                      //Continue until whisker is triggered
  deltaLeaveStartZoneTime = getTimeSince(startingTime);                     //Get the change in time between starting to move and touching the wall 
  stopServos();
  delay(500);                                                               //Robot doesn't do anything
  startServosBackward();                                                    //Goes backward. Prevents the robot from hitting the board.
  delay(800);
  stopServos();                                                             //Robot doesn't do anything
  delay(1000);
  turnPivotLeft();                                                          //Turns left to align robot with board. It is on the side without the cup.
  //go towards edge of board 1st time
  startServosForward();  
  toEdgeOfBoardFirstTime = millis();                                        //Gets the time. This time is when it starts moving foward to 
  while (irRightSensorDetect())                                             //Keep going until the board is there.
  {
    delay(IR_DELAY_TIME);
  }
  
  deltaTimeToFirstCorner = getTimeSince(toEdgeOfBoardFirstTime);            //This saves the time to the first corner from starting near the board.
  alignHitting();
  //got to end of 1st edge, aligned, proceeding to next obstacle side
  turnCorner();
  roundedFirstCornerTime = millis();//saves the time when it first makes it around the 1st corner. may be overwritten if not extreme case 1
  //head towards obstacle first time
  bool isExtremeCase1 = false;
  startServosForward();
   while (getTimeSince(roundedFirstCornerTime) < 1500)
  {
    if(irFrontSensorDetect())
    {
      isExtremeCase1 = true;
      break;
    }
    delay(IR_DELAY_TIME);
  }
  if(!isExtremeCase1)//general case
  {
    alignHittingStart();
    roundedFirstCornerTime=millis();//overwrite the rounded first corner time to after it is aligned
      //not extreme case 1 go forwards as normal
      startServosForward();
    while (!irFrontSensorDetect())
    {
      delay(IR_DELAY_TIME);
    }
  }
   
  deltaFindCupTime = getTimeSince(roundedFirstCornerTime);
  stopServos();
  delay(500);

  //Cup has been detected
  if(deltaFindCupTime < 4500) //for general case (assume there is space to align on other side of the obstacle)
  {
    
    avoidObstacle();     // Avoids obstacle and sets rover parallel to board(using align feature)
    startServosForward(); 
    while (irRightSensorDetect())
    {
      delay(IR_DELAY_TIME);
    }  // While board is there keep going forward
    
    
    alignHitting(); //aligning on right end of board (1st time)
    turnLongCorner();
  }
  else //for extreme case 2 (cup is near end of board)
  {
    avoidObstacleExtremeCase2();//doesn't try to do alignment on right side of obstacle, hard codes around the board to front side of board
  //ends parallel to board past obstacle, but farther away than general case

    startServosForward();//hard code to go past board without alignment
    while (irRightSensorDetect())
    {
      delay(IR_DELAY_TIME);
    }  // While board is there keep going forward
    turnExtraLongCorner();//right turn, forward, right turn. ends parallel to board on front side 
  }

  turnPivotRight();
  tryToHitTheBoard(); // MASON FLAG
  //traverse length of board front side
  startServosForward();
  startMeasuringWallTime = millis(); //to measure length of board time
  while (irRightSensorDetect())
  {
    delay(IR_DELAY_TIME);
  }  // While board is there keep going forward
  
  deltaBoardLengthTime = getTimeSince(startMeasuringWallTime);
  alignHitting();
  //second time around the left corner
  turnCorner();
  roundedThirdCornerTime = millis();
  bool cupFound = true;
//second time around
  if(!isExtremeCase1)//general case
  {
    delay(1500);//move forwards to align (same amount as first time around for general case only)
    stopServos();
    alignHittingStart();
    startServosForward();
    roundedThirdCornerTime = millis(); //rewrite roundedThirdCornerTime if not extreme case 1
  }
  
  while(!irFrontSensorDetect()) //Search for cup
  {
    if (deltaFindCupTime + 1000 < getTimeSince(roundedThirdCornerTime)) // the cup is not there
    {
      delay(500);
      stopServos();
      beepFiveTimes();
      cupFound = false;
      break;
    }
    delay(IR_DELAY_TIME);
    
  }
  
  if (cupFound) // For case: The cup is still there. Move past obstacle and go home
  {
    stopServos();
    beepTwoTimes();

    if(deltaFindCupTime < 6000) //for general case. deltaFindCupTime is constant from first time detecting obstacle
    {
    
      avoidObstacle();     // Avoids obstacle and sets rover parallel to board.
      startServosForward(); 
      while (irRightSensorDetect())
      {
        delay(IR_DELAY_TIME);
      }  // While board is there keep going forward
    
    
      alignHitting(); //TAG
      turnLongCorner();
    }
    else //for extreme case 2 (cup is near end of board)
    {
      avoidObstacleExtremeCase2();
      startServosForward();
      while (irRightSensorDetect())
      {
        delay(IR_DELAY_TIME);
      }  // While board is there keep going forward
      turnExtraLongCorner();
    }
    
    
    turnPivotRight();
    tryToHitTheBoard(); // MASON FLAG
    startServosForward(); 
    lastMeasuringWallTime = millis();
    while(1) // move forwards until perpendicular to the starting zone
    {
      if (deltaBoardLengthTime - deltaTimeToFirstCorner - 500 < getTimeSince(lastMeasuringWallTime)) // offset used to be -750
        break;
    }
    stopServos();
    turnPivotLeft();
    startServosForward(); // return to starting zone
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
    delay(500);
    startServosBackward();
    delay(200);
    stopServos();
  }
  //START
  #endif
  
}
/*
void alignLongHitting()
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
*/

void alignHitting()//for the edge of the board. reverses before aligning
{
  stopServos();
  delay(500);
  startServosBackward();
  delay(600);
  turnPivotBackwardRight();
  stopServos();
  delay(500);
  //startServosBackward();
  //delay(800);
  tryToHitTheBoard(); // MASON FLAG
}

void alignHittingStart()//for the start of the board when not extreme case 1 . reverses before aligning
{
  stopServos();
  delay(500);
  turnPivotBackwardRight();
  stopServos();
  delay(500);
  //startServosBackward();
  //delay(800);
  tryToHitTheBoardSpecialShort(); // special short is for a shorter backup to be closer to the cup when parallel to the board
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
  //delay(1000); // Drive forward so that the sensor will see the board.
}

void turnLongCorner()
{
  startServosForward();
  delay(1000); //Continue Driving forward a bit
  turnPivotRight();
  startServosForward();
  delay(2000); //Get to the other side of the board (used to be 3000)
  turnPivotRight();    
  startServosForward();
  delay(750); // Drive forward so that the sensor will see the board. (used to be 1500)
}

void turnExtraLongCorner()
{
  startServosForward();
  delay(500); //Continue Driving forward a bit (used to be 1000)
  turnPivotRight();
  startServosForward();
  delay(3750); //Get to the other side of the board (used to be 5000)
  turnPivotRight();    
  startServosForward();
  delay(1500); // Drive forward so that the sensor will see the board.(used to be 1500)
}


void avoidObstacle()
{
  //startServosForward();//this is for going a bit forward after detecting the cup
  //delay(200);
  turnPivotLeft();
  startServosForward();
  delay(600);//used to be 1000
  turnPivotRight();
  startServosForward();
  delay(2000);//used to be 2000
  turnPivotRight();
  startServosForward();
  delay(555);//used to be 800
  tryToHitTheBoard();
}

void avoidObstacleExtremeCase2()
{
  startServosForward();
  delay(200);
  turnPivotLeft();
  startServosForward();
  delay(600); // used to be 2000
  turnPivotRight();
  startServosForward();
}
void tryToHitTheBoardBeta()//premise for this function which replaces tryToHitTheBoard is that it reverses until the whiskers don't touch
{
  while(1){
setServosNoFactor(20,20);//go forward slowly
while((!whiskerLeftSensorDetect())&&(!whiskerRightSensorDetect()));//neither whisker touches

delay(100);
stopServos();
bool left = whiskerLeftSensorDetect();
bool right = whiskerRightSensorDetect();
if (left&&right)
{
break; 
}    
else if(left)
{
  //backup and turn left
      setServosNoFactor(-20,-20);//reverse
      while(whiskerLeftSensorDetect());//while left sensor is still detecting
      delay(100);
      setServosNoFactor(-20,20);//turn left
      delay(100);
      stopServos();
} else if(right)
{
  //backup and turn right
      setServosNoFactor(-20,-20);//reverse
      while(whiskerRightSensorDetect());//while left sensor is still detecting
      delay(100);
      setServosNoFactor(20,-20);//turn right
      delay(100);
      stopServos();
}
  
  } 
}
void tryToHitTheBoard()//needs to start facing the board, ends parallel in right direction
{
  setServosNoFactor(20,20);//go forward slowly
  while ((!whiskerLeftSensorDetect())&&(!whiskerRightSensorDetect()));
  stopServos();//stop when either hits
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
      setServosNoFactor(-20,-20);//reverse
      delay(300);
      setServosNoFactor(-20,20);//turn left
      delay(150);//used to be 200
    }
    else if (right)
    {
      setServosNoFactor(-20,-20);//reverse
      delay(300);
      setServosNoFactor(20,-20);//turn right
      delay(150);//used to be 200

    }
    setServosNoFactor(20,20);//go forward
    delay(300);
    
    
  }
  stopServos();
  delay(500);
  startServosBackward(); // Prevents the robot from hitting the board.
  delay(800);//amount of time robot goes back after aligning
  stopServos();
  turnPivotLeft();
}

void tryToHitTheBoardSpecialShort()//special hitting the board for non-extreme case 1. moves back less to be closer to the board when parallel
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
  delay(600);//amount of time robot goes back after aligning
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
  stopServos();
  delay(200);
  startPivotTurningLeft();
  delay(LEFT_PIVOT_TURNING_TIME);
  stopServos();
  delay(200);
}

void turnPivotBackwardLeft()
{
  stopServos();
  delay(200);
  startPivotTurningBackwardLeft();
  delay(RIGHT_PIVOT_TURNING_TIME);
  stopServos();
  delay(200);
}

void startPivotTurningLeft()
{
  setServos(0, 100);
}

void startPivotTurningBackwardLeft()
{
  setServos(-100, 0);
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
  stopServos();
  delay(200);
  startPivotTurningRight();
  delay(RIGHT_PIVOT_TURNING_TIME);
  stopServos();
  delay(200);
}

void turnPivotBackwardRight()
{
  stopServos();
  delay(200);
  startPivotTurningBackwardRight();
  delay(LEFT_PIVOT_TURNING_TIME);
  stopServos();
  delay(200);
}

void startPivotTurningRight()
{
  setServos(100, 0);
}

void startPivotTurningBackwardRight()
{
  setServos(0, -100);
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
