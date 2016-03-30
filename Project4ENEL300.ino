#include <Servo.h>

#define LEFT_SERVO_PIN            11
#define RIGHT_SERVO_PIN           10
#define FRONT_LED_PIN              5
#define FRONT_RECEIVER_PIN        13
#define RIGHT_LED_PIN              3
#define RIGHT_RECEIVER_PIN         4
#define BUZZER_PIN                 9
#define WHISKER_PIN                8
#define WHISKER_LEFT_PIN           8
#define WHISKER_RIGHT_PIN          2
#define VISIBLE_LED_PIN           12

#define RIGHT_DIME_TURNING_TIME  610 
#define LEFT_DIME_TURNING_TIME   630   


#define RIGHT_PIVOT_TURNING_TIME 1310 // CALIBRATION TEST 1 @7.3v
#define LEFT_PIVOT_TURNING_TIME 1250  // CALIBRATION TEST 2 @7.3v
#define IR_DELAY_TIME 10

Servo servoLeft;
Servo servoRight;

double allignment = 0.94; 
//straight alignment factor 0.89@7.3v, 0.95@7.28v

//Higher values make left motor stronger (makes it veer right)
//Lower values make right motor stronger (makes it veer left) 


//The global timing variables
//Note: delta is used to denote changes in time. These are used to determine lengths.
int deltaLeaveStartZoneTime;                                              //start zone to board
int deltaTimeToFirstCorner;                                               //initial contact with board to left end of board
int deltaFindCupTime;                                                     //time from left end to cup 1st time
int deltaBoardLengthTime;                                                 //time to traverse entire side of board (non cup side measured)
  
int startingTime;                                                         //global starting time of entire code
int roundedFirstCornerTime;                                               //time as soon as corner is rounded and robot is parallel to board 1st time
int toEdgeOfBoardFirstTime;
int startMeasuringWallTime;
int roundedThirdCornerTime;
int lastMeasuringWallTime;
int lastStretchTime;
bool isExtremeCase1;
bool isLapOne;
bool cupFound;

int state;
#define ENTRY_STATE 0
#define FIND_THE_BOARD_STATE 1
#define GO_TO_FIRST_EDGE_STATE 2
#define FIRST_ALIGN_HITTING_STATE 3
#define FIND_CUP_STATE 4
#define AVOID_CUP_GENERAL_AND_EXTREME_CASE_ONE 5
#define AVOID_CUP_EXTREME_CASE_TWO 6
#define GO_ACROSS_BOARD_STATE 7
#define TRY_TO_REFIND_CUP_STATE 8
#define REFIND_CUP_EXTREME_CASE_ONE 11
#define REFIND_CUP_GENERAL_CASE_OR_EXTREME_CASE_TWO 12
#define MAKE_CUP_CHOICE_STATE 13
#define CUP_FOUND_STATE 14
#define CUP_NOT_FOUND_STATE 15
#define GO_HOME_STATE 19
#define TERMINATE_STATE 20


void loop() {
  // put your setup code here, to run once:
  //STARTUP

  pinMode(VISIBLE_LED_PIN, OUTPUT);
  //ROBOT: goes from the starting board to the board.
  switch(state)
  {
    case ENTRY_STATE:
      delay(1000);                                                              // Gives a bit of time for us to position the robot before it starts moving
      attachMotors();
      isExtremeCase1 = false;
      isLapOne = true;
      state = FIND_THE_BOARD_STATE;
      break;
    case FIND_THE_BOARD_STATE:
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
      state = GO_TO_FIRST_EDGE_STATE;
      break;
    case GO_TO_FIRST_EDGE_STATE:
      startServosForward();  
      toEdgeOfBoardFirstTime = millis();                                        //Gets the time. This time is when it starts moving foward to 
      while (irRightSensorDetect())                                             //Keep going until the board is there.
      {
        delay(IR_DELAY_TIME);
      }                                                                         //go towards edge of board 1st time
      deltaTimeToFirstCorner = getTimeSince(toEdgeOfBoardFirstTime);            //This saves the time to the first corner from starting near the board.
      alignHitting();
      //ROBOT: got to end of 1st edge, aligned, proceeding to next obstacle side
      turnCorner();
      state = FIND_CUP_STATE;
      break;
    case FIND_CUP_STATE: 
      startServosForward();
      roundedFirstCornerTime = millis();//saves the time when it first makes it around the 1st corner. may be overwritten if not extreme case 1
     
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
      
      if(deltaFindCupTime < 4500) //for general case (assume there is space to align on other side of the obstacle)
      {
        state = AVOID_CUP_GENERAL_AND_EXTREME_CASE_ONE;
      }
      else //for extreme case 2 (cup is near end of board)
      {
        state = AVOID_CUP_EXTREME_CASE_TWO;
      }

      break;
    case AVOID_CUP_GENERAL_AND_EXTREME_CASE_ONE:   
      avoidObstacle();     // Avoids obstacle and sets rover parallel to board(using align feature)
      startServosForward(); 
      while (irRightSensorDetect())
      {
        delay(IR_DELAY_TIME);
      }  // While board is there keep going forward
      
      
      alignHitting(); //aligning on right end of board (1st time)
      turnLongCorner();
      if (isLapOne)
        state = GO_ACROSS_BOARD_STATE;
      else
        state = GO_HOME_STATE;
      break;
    case AVOID_CUP_EXTREME_CASE_TWO:
      avoidObstacleExtremeCase2();//doesn't try to do alignment on right side of obstacle, hard codes around the board to front side of board
      //ends parallel to board past obstacle, but farther away than general case

      startServosForward();//hard code to go past board without alignment
      while (irRightSensorDetect())
      {
        delay(IR_DELAY_TIME);
      }  // While board is there keep going forward
      turnExtraLongCorner();//right turn, forward, right turn. ends parallel to board on front side 
      if (isLapOne)
        state = GO_ACROSS_BOARD_STATE;
      else
        state = GO_HOME_STATE;
      break;
    case GO_ACROSS_BOARD_STATE:
      turnPivotRight();
      alignAndTurn();
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
      turnCorner();//goes forwards after turning corner
      isLapOne = false;
      state = TRY_TO_REFIND_CUP_STATE;
      break;
    case TRY_TO_REFIND_CUP_STATE:
      roundedThirdCornerTime = millis();
      cupFound = true;
      //second time around
      if(!isExtremeCase1)//general case
      {
        state = REFIND_CUP_GENERAL_CASE_OR_EXTREME_CASE_TWO;
      }
      else
      {//for extremecase1
        //if cup not found within deltaFindCupTime + 1000, then cup not found
        state = REFIND_CUP_EXTREME_CASE_ONE;
      }
      break;

    case CUP_FOUND_STATE:
      stopServos();
      beepTwoTimes();

      if(deltaFindCupTime < 4500) //for general case. deltaFindCupTime is constant from first time detecting obstacle
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
      
      //asdfafsddsf
      state = GO_HOME_STATE;
      break;

    case REFIND_CUP_EXTREME_CASE_ONE:
      while(!irFrontSensorDetect()) //Search for cup
      {
        if (deltaFindCupTime + 1000 < getTimeSince(roundedThirdCornerTime)) // the cup is might not be there
        {
          cupFound=false; 
          break;
        }     
      delay(IR_DELAY_TIME);
      }
      state = MAKE_CUP_CHOICE_STATE;
      break;

    case REFIND_CUP_GENERAL_CASE_OR_EXTREME_CASE_TWO:
      delay(1500);//move forwards to align (same amount as first time around for general case only)
      stopServos();
      alignHittingStart();
      startServosForward();
      roundedThirdCornerTime = millis(); //rewrite roundedThirdCornerTime if not extreme case 1

      //******
      while(!irFrontSensorDetect()) //Search for cup
      {
        if (deltaFindCupTime + 1000 < getTimeSince(roundedThirdCornerTime)) // the cup is might not be there
        {
          cupFound=false;
          //reverse and realign just to double check that the cup is gone
          if(deltaFindCupTime>3000)
          {//## if(too close for a double check)
            startServosBackward();
            delay(1000); //reverse 1000
            stopServos();
            delay(500);
            turnPivotBackwardRight();//turn towards the board
            alignAndTurn();//align
            //search for another 1500
            int doubleCheckTime=millis();//save time right after alignment
            startServosForward();
            while(getTimeSince(doubleCheckTime)<1500)
            {//give it a chance to find the cup in 1500ms
              if(irFrontSensorDetect())
              {
                cupFound=true;
                break;
              }
            }
            break;
          }
          else
          {// end the if statement ## 
            break;
          }
        }
        delay(IR_DELAY_TIME);
      }
      state = MAKE_CUP_CHOICE_STATE;
      break;

    case MAKE_CUP_CHOICE_STATE:
      if (cupFound) // For case: The cup is still there. Move past obstacle and go home
        state = CUP_FOUND_STATE;
      else
        state = CUP_NOT_FOUND_STATE;
      break;

    case CUP_NOT_FOUND_STATE:
      delay(500);//allows it to get nearer to the cup
      stopServos();
      beepFiveTimes();
      state = TERMINATE_STATE;
      break;

    case GO_HOME_STATE:
      turnPivotRight();
      alignAndTurn(); //MASON FLAG
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
      //This code uses the front sensors to determine when it has hit the backdrop and is home.
      while (!whiskerFrontSensorDetect());    //Continue until whisker is triggered
      stopServos();
      delay(500);
      startServosBackward();
      delay(200);
      stopServos();
      state = TERMINATE_STATE;
      break;
    case TERMINATE_STATE:
      stopServos();
      break;

      
  default:
    // Problem: Blink Light 5 times
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(VISIBLE_LED_PIN, HIGH);
      delay(300);
      digitalWrite(VISIBLE_LED_PIN, LOW);
      delay(300);
    }
    break;




 } //End of switch


  //ROBOT: head towards obstacle first time
  

  //Cup has been detected
  


  
}

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
  alignAndTurn(); // MASON FLAG
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
  alignAndTurn(); // special short is for a shorter backup to be closer to the cup when parallel to the board
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
  alignAndTurn();
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
  while(1)
  {
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
    } 
    else if(right)
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
void alignAndTurn()
{
  tryToHitTheBoardBeta();
  delay(500);
  startServosBackward(); // back up after aligning
  delay(800);
  stopServos();
  turnPivotLeft();
  delay(100);
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
      delay(80);
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
  int ir = !digitalRead(irReceiverPin);      // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}  



void setup()
{
  state = ENTRY_STATE;
}
