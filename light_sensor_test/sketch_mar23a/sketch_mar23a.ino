void setup()
// Built-in initialization block // Set data rate to 9600 bps
// Main loop auto-repeats
{ 
  
Serial.begin(9600);
  }
void loop() {
//learn.parallax.com/print/book/export/html/114
//138/206
//12/2/13
//learn.parallax.com/print/book/export/html/114
// Display "A3 = "
// Display measured A3 volts
// Display " volts" & newline // Delay for 1 second
// Measures volts at adPin

Serial.print("A0 = "); 
Serial.print(volts(A0)); 
Serial.println(" volts"); delay(500);
 }
float volts(int adPin) {
// Returns floating point voltage 
return float(analogRead(adPin)) * 5.0 / 1024.0;
}
