#include <Servo.h>
int ledout=6;
/*
 * Robotics with the BOE Shield - TestLeftIR
 * Display 1 if the left IR detector does not detect an object, 
 * or 0 if it does. 
 */ 
Servo left, right;
void setup()                                 // Built-in initialization block
{ 
  pinMode(ledout, OUTPUT);
  
  left.attach(11);
  right.attach(10);
//  right.writeMicroseconds(1300);
//  left.writeMicroseconds(1700);
  tone(2, 38000, 8);                       // Play tone for 1 second
  delay(1000);                               // Delay to finish tone

  pinMode(4, INPUT);  pinMode(3, OUTPUT);   // Left IR LED & Receiver

  Serial.begin(9600);                        // Set data rate to 9600 bps
}  
 
void loop()                                  // Main loop auto-repeats
{
  int irLeft = irDetect(3, 4, 38000);       // Check for object
  digitalWrite(ledout, LOW);
if(irLeft==1)
{
//  left.writeMicroseconds(1500);
//  right.writeMicroseconds(1500);
digitalWrite(ledout, HIGH);
//delay(00);
}
  
  Serial.println(irLeft);                    // Display 1/0 no detect/detect

  delay(50);                                // 0.1 second delay
} 

// IR Object Detection Function

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = !digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}  

