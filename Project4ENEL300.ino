#include <Servo.h>

#define LEFT_SERVO_PIN                       11
#define RIGHT_SERVO_PIN                      10
#define FRONT_LED_PIN                         5
#define FRONT_RECEIVER_PIN                   13
#define RIGHT_LED_PIN                         3
#define RIGHT_RECEIVER_PIN                    4
#define BUZZER_PIN                            9
#define WHISKER_PIN                           8
#define WHISKER_LEFT_PIN                      8
#define WHISKER_RIGHT_PIN                     2
#define VISIBLE_LED_PIN                      12

#define RIGHT_DIME_TURNING_TIME             610 
#define LEFT_DIME_TURNING_TIME              630   


#define RIGHT_PIVOT_TURNING_TIME           1310 // CALIBRATION TEST 1 @7.3v
#define LEFT_PIVOT_TURNING_TIME            1250 // CALIBRATION TEST 2 @7.3v
#define IR_DELAY_TIME                        10

Servo servoLeft;
Servo servoRight;

double alignment = 0.92;                       //straight alignment factor 0.89@7.3v, 0.95@7.28v
                                                                                //Higher values make left motor stronger (makes it veer right)
                                                                                //Lower values make right motor stronger (makes it veer left) 


//The global timing variables
//Note: delta is used to denote changes in time. These are used to determine lengths.


int deltaLeaveStartZoneTime;                                                    //Start zone to board
int deltaTimeToFirstCorner;                                                     //Initial contact with board to left end of board
int deltaFindCupTime;                                                           //Time from left end to cup 1st time
int deltaBoardLengthTime;                                                       //Time to traverse entire side of board (non cup side measured)
  
int startingTime;                                                               //Global starting time of entire code
int roundedFirstCornerTime;                                                     //Time as soon as corner is rounded and robot is parallel to board 1st time
int toEdgeOfBoardFirstTime;
int startMeasuringWallTime;
int roundedThirdCornerTime;
int lastMeasuringWallTime;
int lastStretchTime;
bool isExtremeCase1;
bool isLapOne;
bool cupFound;

int state;
#define ENTRY_STATE                                  0
#define FIND_THE_BOARD_STATE                         1
#define GO_TO_FIRST_EDGE_STATE                       2
#define FIRST_ALIGN_HITTING_STATE                    3
#define FIND_CUP_STATE                               4
#define AVOID_CUP_GENERAL_AND_EXTREME_CASE_ONE       5
#define AVOID_CUP_EXTREME_CASE_TWO                   6
#define GO_ACROSS_BOARD_STATE                        7
#define TRY_TO_REFIND_CUP_STATE                      8
#define REFIND_CUP_EXTREME_CASE_ONE                  9
#define REFIND_CUP_GENERAL_CASE_OR_EXTREME_CASE_TWO 10
#define CUP_FOUND_STATE                             11
#define CUP_NOT_FOUND_STATE                         12
#define GO_HOME_PART_A_STATE                        13
#define GO_HOME_PART_B_STATE                        14
#define TERMINATE_STATE                             15

void setup()                                                                    //On setup, go to the entry state.
{
  state = ENTRY_STATE;
}

void loop() {
  pinMode(VISIBLE_LED_PIN, OUTPUT);                                             //Sets LED to output. Used for debugging
  //ROBOT: goes from the starting board to the board.
  switch(state)
  {
    case ENTRY_STATE:
    //do: short pause
      delay(1000);                                                              //Gives a bit of time for us to position the robot before it starts moving
      attachMotors();
      isExtremeCase1 = false;
      isLapOne = true;
      state = FIND_THE_BOARD_STATE;
      break;
    case FIND_THE_BOARD_STATE:
      startingTime = millis();                                                  //Time at the begining of the test
      //Do: check for board and go forward
      startServosForward();                                                     //Starts going forward towards board
      while (!whiskerFrontSensorDetect());                                      //Continue until whisker is triggered
      deltaLeaveStartZoneTime = getTimeSince(startingTime);                     //Get the change in time between starting to move and touching the wall 
      stopServos();
      //Exit: Turn left
      delay(500);                                                               //Robot doesn't do anything
      startServosBackward();                                                    //Goes backward. Prevents the robot from hitting the board.
      delay(800);
      stopServos();                                                             //Robot doesn't do anything
      delay(1000);
      turnPivotLeft();                                                          //Turns left to align robot with board. It is on the side without the cup.
      toEdgeOfBoardFirstTime = millis(); 
      state = GO_TO_FIRST_EDGE_STATE;
      break;
    case GO_TO_FIRST_EDGE_STATE:
      //do: move forward and check for edge of board
      startServosForward();                                                     //Gets the time. This time is when it starts moving foward to 
      checkForEdgeOfBoard();                                                    //Go towards edge of board 1st time
      deltaTimeToFirstCorner = getTimeSince(toEdgeOfBoardFirstTime);            //This saves the time to the first corner from starting near the board.
      alignHitting();
      //ROBOT: got to end of 1st edge, aligned, proceeding to next obstacle side
      turnCorner();
      state = FIND_CUP_STATE;
      break;
    case FIND_CUP_STATE: 
      startServosForward();
      roundedFirstCornerTime = millis();                                        //Saves the time when it first makes it around the 1st corner. may be overwritten if not extreme case 1
      checkForExtremeCase1();
      if(!isExtremeCase1)                                                       //Align if not extreme case 1. We thought this is unique enough that it does not require a state
      {
        alignHittingStart();
        roundedFirstCornerTime=millis();                                        //Overwrite the rounded first corner time to after it is aligned
                                                                                //Not extreme case 1 go forwards as normal
        startServosForward();
        while (!irFrontSensorDetect())
        {
          delay(IR_DELAY_TIME);                                                       //Delays a bit so irSensor does not lead to false readings.
        }
      }
      
      deltaFindCupTime = getTimeSince(roundedFirstCornerTime);
      stopServos();
      delay(500);
      
      if(deltaFindCupTime < 4500)                                               //For general case (assume there is space to align on other side of the obstacle)
      {
        state = AVOID_CUP_GENERAL_AND_EXTREME_CASE_ONE;
      }
      else                                                                      //For extreme case 2 (cup is near end of board)
      {
        state = AVOID_CUP_EXTREME_CASE_TWO;
      }

      break;
    case AVOID_CUP_GENERAL_AND_EXTREME_CASE_ONE:   
      avoidObstacle();                                                          //Avoids obstacle and sets rover parallel to board(using align feature)
      startServosForward();                                                     
      checkForEdgeOfBoard();                                                    //Do nothing until reaches edge of the board
      alignHitting();                                                           //Aligning on right end of board (1st time)
      turnLongCorner();
      if (isLapOne)
        state = GO_ACROSS_BOARD_STATE;
      else
        state = GO_HOME_PART_A_STATE;
      break;
    case AVOID_CUP_EXTREME_CASE_TWO:
      avoidObstacleExtremeCase2();                                              //Doesn't try to do alignment on right side of obstacle, hard codes around the board to front side of board
                                                                                //Ends parallel to board past obstacle, but farther away than general case

      startServosForward();                                                     //Hard code to go past board without alignment
      checkForEdgeOfBoard();                                                    //Do nothing until reaches edge of the board
      turnExtraLongCorner();                                                    //Right turn, forward, right turn. ends parallel to board on front side 
      if (isLapOne)
        state = GO_ACROSS_BOARD_STATE;
      else
        state = GO_HOME_PART_A_STATE;
      break;
    case GO_ACROSS_BOARD_STATE:
      turnPivotRight();
      alignAndTurn();                                                           //Turn towards the board and align.
      //traverse length of board front side
      startServosForward();                                                     //Go forward accross the board
      startMeasuringWallTime = millis();                                        //to measure length of board time
      checkForEdgeOfBoard();                                                    //Do nothing until reaches edge of the board
      deltaBoardLengthTime = getTimeSince(startMeasuringWallTime);              //Calculates time. Used to determine size of the board.
      alignHitting();                                                           //Realign at the other side of the board.
      //second time around the left corner
      turnCorner();                                                             //Turns around corner
      isLapOne = false;                                                         //This is not the first lap. (Second Lap)
      state = TRY_TO_REFIND_CUP_STATE;
      break;
    case TRY_TO_REFIND_CUP_STATE:
      roundedThirdCornerTime = millis();                                        //Start timer after turning around corner. This is used to determine distance to the cup.
      cupFound = true;                                                          //Assume the cup is found. We will check for the counter case in the REFIND_CUP_*CASE_* states.
      //second time around board
      if(!isExtremeCase1)                                                       //If it is a general case or extreme case two
      {
        state = REFIND_CUP_GENERAL_CASE_OR_EXTREME_CASE_TWO;
      }
      else
      {                                                                         //For extremecase1
        state = REFIND_CUP_EXTREME_CASE_ONE;                                    //If cup not found within deltaFindCupTime + 1000, then cup not found
      }
      break;

    case CUP_FOUND_STATE:
      stopServos();                                                             //The cup has been found. Stop.
      beepTwoTimes();                                                           //Beep two times...............Object still there

      if(deltaFindCupTime < 4500)                                               //Ror general case. deltaFindCupTime is constant from first time detecting obstacle
      {
        avoidObstacle();                                                        //Avoids obstacle and sets rover parallel to board.
        startServosForward(); 
        checkForEdgeOfBoard();                                                  //Do nothing until reaches edge of the board
        alignHitting();                                                         //Update alignment
        turnLongCorner();                                                       //Turn around the corner, but go a bit longer so that we can easily realign on the other side.
      }
      else //for extreme case 2 (cup is near end of board)
      {
        avoidObstacleExtremeCase2();                                            //The cup is at the end of the board. Do not even try to come back to the board. Go around the board instead.
        startServosForward();
        checkForEdgeOfBoard();                                                  //Do nothing until reaches edge of the board
        turnExtraLongCorner();                                                  //Turn around the corner, but go a bit longer so that we can easily realign on the other side.
      }
      
      //asdfafsddsf
      state = GO_HOME_PART_A_STATE;
      break;

    case REFIND_CUP_EXTREME_CASE_ONE:
      refindCupExtremeCase1();
      if (cupFound)                                                             //For case: The cup is still there. Move past obstacle and go home
        state = CUP_FOUND_STATE;
      else
        state = CUP_NOT_FOUND_STATE;
      break;

    case REFIND_CUP_GENERAL_CASE_OR_EXTREME_CASE_TWO:
      delay(1500);                                                              //Move forwards to align (same amount as first time around for general case only)
      stopServos();
      alignHittingStart();                                                      //Align hitting to make sure that we are aligned
      startServosForward();
      roundedThirdCornerTime = millis();                                        //Rewrite roundedThirdCornerTime if not extreme case 1

      //******
      checkForCupGeneralCase();
      if (cupFound)                                                             //For case: The cup is still there. Move past obstacle and go home
        state = CUP_FOUND_STATE;
      else
        state = CUP_NOT_FOUND_STATE;
      break;

    case CUP_NOT_FOUND_STATE:
      delay(500);                                                               //Allows it to get closer to where the cup was.
      stopServos();
      beepFiveTimes();                                                          //Beep five times then terminate.
      state = TERMINATE_STATE;
      break;

    case GO_HOME_PART_A_STATE:
      turnPivotRight();                                                         //The cup was refound and we are on the other side of the board (the side without the cup)
                                                                                //Turn toward the board and align.
      alignAndTurn();
      startServosForward(); 
      lastMeasuringWallTime = millis();
      while(1)                                                                  //Move forwards until perpendicular to the starting zone
      {
        if (deltaBoardLengthTime - deltaTimeToFirstCorner - 500 < getTimeSince(lastMeasuringWallTime)) // offset used to be -750. This line moves the robot until the robot at the closest point to the starting area, while still being parallel to the board.
          break;
      }
      stopServos();
      turnPivotLeft();                                                          //Pivot toward the starting area.
      state = GO_HOME_PART_B_STATE;
      break;
    case GO_HOME_PART_B_STATE:
      startServosForward();
      lastStretchTime = millis();                                               //Originally, we were implementing a timing mechanism to get the robot home. We no longer use this.
                                                                                //This code uses the front sensors to determine when it has hit the backdrop and is home.
      while (!whiskerFrontSensorDetect());                                      //Continue until whisker is triggered. Unique enough not to be moved into a function
      stopServos();                                                             //Stop the servos and then back up a bit so that we are in the middle of the square.
      delay(500);
      startServosBackward(); 
      delay(200);
      stopServos();
      state = TERMINATE_STATE;
      break;
    case TERMINATE_STATE:                                                       //This state is used to terminate.
      stopServos();
      break;

      
  default:
    // If there is a problem with the state machine: Blink Light 5 times
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(VISIBLE_LED_PIN, HIGH);
      delay(300);
      digitalWrite(VISIBLE_LED_PIN, LOW);
      delay(300);
    }
    break;
 } //End of switch
} //End of loop

void alignHitting()                                                             //For the edge of the board. reverses before aligning
{
  stopServos();                                                                 //Stop the servos. Robot is parallel to board
  delay(500);
  startServosBackward();                                                        //Reverse a bit. so that when it turns, both whiskers will touch the board.
  delay(600);
  turnPivotBackwardRight();                                                     //Pivot backwards to the right, so that robot faces the board.
  stopServos();
  delay(500);
  alignAndTurn();                                                               //Start alignment procedure. After, turn so that parallel with the board again.
}

void alignHittingStart()                                                        //For the start of the board when not extreme case 1 . reverses before aligning
{
  stopServos();
  delay(500);
  turnPivotBackwardRight();                                                     //Turn right by pivoting backward.
  stopServos();
  delay(500);
  alignAndTurn();                                                               //Special short is for a shorter backup to be closer to the cup when parallel to the board
}


void turnCorner()                                                               //Turn around corner.
{
  startServosForward();                                                         //Go forward. Robot is parallel to the board
  delay(1000);                                                                  //Continue Driving forward a bit
  turnPivotRight();                                                             //Pivot and go foward. Robot is now perpendicular to the board.
  startServosForward();
  delay(1500);                                                                  //Get to the other side of the board
  turnPivotRight();                                                             //Pivot so robot is parallel with the board.
  startServosForward();                                                         //Start going forward.
}

void turnLongCorner()                                                           //Turns around corner. Starts going forward
{
  startServosForward(); 
  delay(1000);                                                                  //Continue Driving forward a bit
  turnPivotRight();                                                             //Pivot and go foward. Robot is now perpendicular to the board.
  startServosForward();
  delay(2000);                                                                  //Get to the other side of the board (used to be 3000)
  turnPivotRight();                                                             //Pivot so robot is parallel with the board.
  startServosForward();
  delay(750);                                                                   //Drive forward so that the sensor will see the board. (used to be 1500)
}

void turnExtraLongCorner()
{
  startServosForward();
  delay(500);                                                                   //Continue Driving forward a bit (used to be 1000)
  turnPivotRight();
  startServosForward();
  delay(3750);                                                                  //Get to the other side of the board (used to be 5000)
  turnPivotRight();    
  startServosForward();
  delay(1500);                                                                  // Drive forward so that the sensor will see the board.(used to be 1500)
}


void avoidObstacle()                                                            //Used to avoid the obstacle in general case and extreme case 1
{                                                                               //At this point, the robot is parallel with the board.
  turnPivotLeft();                                                              //Pivot left avoiding the obsticle. Robot perpendicular to board.
  startServosForward();                                                         //Go forward a bit making a box around cup.
  delay(600);                                                                   //used to be 1000
  turnPivotRight();                                                             //Pivot to the right. Robot now parallel with the board.
  startServosForward();                                                         //Go forward around obstacle.
  delay(2500);                                                                  //used to be 2000. time to go forward to clear cup
  turnPivotRight();                                                             //Turn right. The Robot will now be faceing the board.
  startServosForward();                                                         //Go forward going towards the board.
  delay(555);//used to be 800
  alignAndTurn();                                                               //Try to align and then turn back on to the path.
}

void avoidObstacleExtremeCase2()                                                //Used when the cup is really close to the edge of the board on the far side. 
{                                                                               //There will not be time to align with the board after seeing the cup.
  startServosForward();                                                         //Go forward a bit towards cup.
  delay(200);
  turnPivotLeft();                                                              //Turn left away from cup and faceing away from board. 
  startServosForward();                                                         //Go forward around box
  delay(600);                                                                   //Used to be 2000
  turnPivotRight();                                                             //Turn so parallel to board going in proper direction.
  startServosForward();                                                         //Go forward. Later function will stop the robot when the board is no longer detected.
}

void tryToHitTheBoardBeta()                                                     //Premise for this function which replaces tryToHitTheBoard is that it reverses until the whiskers don't touch
{
  while(1)
  {
    setServosNoFactor(20,20);                                                   //Go forward slowly
    while((!whiskerLeftSensorDetect())&&(!whiskerRightSensorDetect()));         //While neither whisker touches continue going forward
    delay(100);
    stopServos();
    bool left = whiskerLeftSensorDetect();
    bool right = whiskerRightSensorDetect();
    if (left&&right)                                                            //If the left and the right whiskers are touching at the same time, alignment is done so break.
    {
      break; 
    }    
    else if(left)                                                               //If the left sensor is triggered first, need to backup and turn left
    {
      setServosNoFactor(-20,-20);                                               //Reverse
      while(whiskerLeftSensorDetect());                                         //While left sensor is still detecting keep going backward
      delay(100);
      setServosNoFactor(-20,20);                                                //Turn left
      delay(100);
      stopServos();
    } 
    else if(right)                                                              //If the right sensor is triggered first, need to backup and right left
    {
      setServosNoFactor(-20,-20);                                               //Reverse
      while(whiskerRightSensorDetect());                                        //While left sensor is still detecting
      delay(100);
      setServosNoFactor(20,-20);                                                //Turn right
      delay(100);
      stopServos();
    }
  
  } 
}

void alignAndTurn()
{
  tryToHitTheBoardBeta();                                                       //Try to hit the board and align.
  delay(500);
  startServosBackward();                                                        //Back up after aligning
  delay(800);
  stopServos();
  turnPivotLeft();                                                              //Turn left so that parallel to the board, faceing the proper direction
  delay(100);
}

void attachMotors()                                                             //Attaches the motors so that they can be used.
{
  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);
}

void stopServos()                                                               //Sends stop command to Servos.
{
  setServos(0, 0);
}

void startServosForward()                                                       //Sends full forward (in linear region) command to Servos.
{
  setServos(100, 100);
}

void startServosBackward()                                                      //Sends backward forward (in linear region) command to Servos.
{
  setServos(-100, -100);
}

int getTimeSince(int input)                                                     //In milliseconds
{
  return millis() - input;
}

void turnDimeLeft()                                                             //Turns on a dime. Never used
{
  startDimeTurningLeft();
  delay(LEFT_DIME_TURNING_TIME);
  stopServos();
}

void turnPivotLeft()                                                            //Pivots left for a specified time
{
  stopServos();
  delay(200);
  startPivotTurningLeft();
  delay(LEFT_PIVOT_TURNING_TIME);                                               //The left motor time is used as the left motor is used.
  stopServos();
  delay(200);
}

void turnPivotBackwardLeft()                                                    //Pivots backward left for a specified time
{
  stopServos();
  delay(200);
  startPivotTurningBackwardLeft();
  delay(RIGHT_PIVOT_TURNING_TIME);                                              //The right motor time is used as the right motor is used.
  stopServos();
  delay(200);
}

void startPivotTurningLeft()                                                    //Command to start pivoting left
{
  setServos(0, 100);
}

void startPivotTurningBackwardLeft()                                            //Command to start pivoting backward left
{
  setServos(-100, 0);
}

void turnDimeRight()                                                            //Turns on a dime. Never used
{
  startDimeTurningRight();
  delay(RIGHT_DIME_TURNING_TIME);
  stopServos();
}

void startDimeTurningLeft()                                                     //Start turning on a dime. Never used
{
  setServos(-100, 100);
}

void turnPivotRight()                                                           //Pivots left for a specified time
{
  stopServos();
  delay(200);
  startPivotTurningRight();
  delay(RIGHT_PIVOT_TURNING_TIME);                                              //The right motor time is used as the right motor is used.
  stopServos();
  delay(200);
}

void turnPivotBackwardRight()                                                   //Command to start pivoting backward right
{
  stopServos();
  delay(200);
  startPivotTurningBackwardRight();
  delay(LEFT_PIVOT_TURNING_TIME);                                               //The left motor time is used as the left motor is used.
  stopServos();
  delay(200);
}

void startPivotTurningRight()                                                   //Command to start pivoting right
{
  setServos(100, 0);
}

void startPivotTurningBackwardRight()                                           //Command to start pivoting backwards right
{
  setServos(0, -100);
}

void startDimeTurningRight()                                                    //Start turning on a dime. Never used
{
  setServos(100, -100);
}

/*
  setServos allows one to use positive offsets from 1500 (stop). 
  The convention is that positive values will move the robot forward and
  negative values will move the robot backwards.
  */
  
void setServos(int left, int right)                                             //Convention is Positive numbers for forward
{
  servoLeft.writeMicroseconds(1500+(double)left*alignment);                     //Positive for forward, negative for backward
  servoRight.writeMicroseconds(1500-(double)right/alignment);                   //Negative for forward, positive for backward
}

void setServosNoFactor(int left, int right)                                     //Convention is Positive numbers for forward
{
  servoLeft.writeMicroseconds(1500+(double)left);                               //Positive for forward, negative for backward
  servoRight.writeMicroseconds(1500-(double)right);                             //Negative for forward, positive for backward
}

void beepTwoTimes()
{
  int i;
  for(i = 0; i < 2; i++)
  {
    tone(BUZZER_PIN, 1000, 500);                                                //Start tone
    delay(1000);                                                                //Pause for a second
    noTone(BUZZER_PIN);                                                         //No tone
  }
}

void beepFiveTimes()
{
  int i;
  for(i = 0; i < 5; i++)
  {
    tone(BUZZER_PIN, 1000, 500);                                                //Start tone
    delay(1000);                                                                //Pause for a second
    noTone(BUZZER_PIN);                                                         //No tone
  }
}

boolean whiskerFrontSensorDetect()
{
  return (whiskerLeftSensorDetect() || whiskerRightSensorDetect());             //When the pin goes low, the whisker has touched.
}

boolean whiskerLeftSensorDetect()
{
  return !digitalRead(WHISKER_LEFT_PIN);                                        //When the pin goes low, the whisker has touched.
}

boolean whiskerRightSensorDetect()
{
  return !digitalRead(WHISKER_RIGHT_PIN);                                       //When the pin goes low, the whisker has touched.
}

boolean irFrontSensorDetect()
{
  return irSensorDetect(FRONT_LED_PIN, FRONT_RECEIVER_PIN);                     //True when the front ir sensor is detecting
}

boolean irRightSensorDetect()
{
  return irSensorDetect(RIGHT_LED_PIN, RIGHT_RECEIVER_PIN);                     //True when the right ir sensor is detecting
}

boolean irSensorDetect(int irLedPin, int irReceiverPin)                         //Tests many inputs of the ir sensor to manage false positives
{
  int count = 0;
  for (int i = 0; i < 20; i++)                                                  //Take 20 samples with the ir sensor
  {
    count += irDetect(irLedPin, irReceiverPin, 38000);
    delay(1);
  } 
  if (count > 15)                                                               
    return true;                                                                //If 15 are true, detection occured. Otherwise, no detection.
  return false;
}


int irDetect(int irLedPin, int irReceiverPin, long frequency)                   //This function is from the textbook
{
  tone(irLedPin, frequency, 8);                                                 // IRLED 38 kHz for at least 1 ms
  delay(1);                                                                     // Wait 1 ms
  int ir = !digitalRead(irReceiverPin);                                         // IR receiver -> ir variable
  delay(1);                                                                     // Down time before recheck
  return ir;                                                                    // Return 1 no detect, 0 detect
}  


void checkForEdgeOfBoard()
{
  while (irRightSensorDetect())                                                 //Keep going until the board is there.
  {
    delay(IR_DELAY_TIME);                                                       //Delays a bit so irSensor does not lead to false readings.
  }     
}

void checkForExtremeCase1()                                                     //Checks to see if Extreme Case 1. This occurs if the cup is really close to the corner of the board where the robot first faces the cup.
{
  while (getTimeSince(roundedFirstCornerTime) < 1500)                           //If the time since rounding the corner is less than this time
  {
    if(irFrontSensorDetect())                                                   //isExtremeCase1 = true if the cup is seen. Else, it is not extreme case 1.
    {
      isExtremeCase1 = true;
      break;
    }
    delay(IR_DELAY_TIME);                                                       //Delays a bit so irSensor does not lead to false readings.
  }
}

void refindCupExtremeCase1()                                                    //If we are refinding the cup in the extreme case 1, we use this function.
{
  while(!irFrontSensorDetect())                                                 //Search for cup
  {
    if (deltaFindCupTime + 1000 < getTimeSince(roundedThirdCornerTime))         //If the timer expires, break. The cup is not there.
    {
      cupFound=false; 
      break;
    }     
  delay(IR_DELAY_TIME);                                                         //Delays a bit so irSensor does not lead to false readings.
  }
}

void checkForCupGeneralCase()
{
  while(!irFrontSensorDetect())                                                 //Search for cup
  {
    if (deltaFindCupTime + 1000 < getTimeSince(roundedThirdCornerTime))         //If the timer expires while the cup is not there, the cup has not been found yet. We will do another check after alignment.
    {
      cupFound=false;                                                           //The cup has "not been found".
                                                                                //Reverse and realign just to double check that the cup is gone
      if(deltaFindCupTime>3000)                                                 //If the time to the cup, was over 3 seconds, go back and check again. Else, the cup really is not there.
      {
        startServosBackward();
        delay(1000);                                                            //Reverse 1000
        stopServos();
        delay(500);
        turnPivotBackwardRight();                                               //Turn towards the board
        alignAndTurn();                                                         //Align and then turn so parallel to the board.
        //search for another 1500
        int doubleCheckTime=millis();                                           //save time right after alignment
        startServosForward();
        while(getTimeSince(doubleCheckTime)<1500)                               //Look for the cup for the next 1500ms
        {
          if(irFrontSensorDetect())                                             //If the cup is found, set cupFound to true then break.
          {
            cupFound=true;
            break;
          }
        }
        break;                                                                  //The cup has not been found. Break.
      } //End of if (deltaFindCupTime>3000)
      else
      {
        break;                                                                  //If the cup was not originally over 3000ms away, break. The cup is not there.
      }
    }
    delay(IR_DELAY_TIME);                                                       //Delays a bit so irSensor does not lead to false readings.
  }
}
