#include <Servo.h>

Servo servo1; // Y-Axis (Tilt) - Pin 6
Servo servo2; // X-Axis (Pan)  - Pin 9

void setup() {
  // Using baud rate 115200 -> faster than 9600
  Serial.begin(115200);
  
  // I set the timeout to 5ms -> Arduino don't wait longer for the data (default is 1000ms)
  Serial.setTimeout(5); 

  servo1.attach(6); 
  servo2.attach(9);

  // Default Position (Center)
  servo1.write(90);
  servo2.write(90);
  
  // Quick LED flash to confirm it's ready
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); delay(200); digitalWrite(13, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data until the newline character
    String data = Serial.readStringUntil('\n');
    
    // Parse the "X,Y" format
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      String xStr = data.substring(0, commaIndex);
      String yStr = data.substring(commaIndex + 1);
      

      int valX = xStr.toInt();
      int valY = yStr.toInt();
      
      /* WARNING: PLEASE ADJUST THE LOGIC YOURSELF
    Because it depends on your real setup: brackets, load,
    and many possible servo stall cases (it can hurt your servo, seriously).
    */


      // --- SERVO 1 (Y / TILT) Logic ---
      // I set the threshold to 15 here (slightly more sensitive than X).
      if (valY > 15) {
        servo1.write(60);   // Change position
      } else if (valY < -15) {
        servo1.write(135);  // Change position
      } else {
        servo1.write(90);   // Back to center
      }

      // --- SERVO 2 (X / PAN) Logic ---
      // Threshold is 20.
      if (valX > 20) {
        servo2.write(0);  // Change position
      } else if (valX < -20) {
        servo2.write(180);    // Change position
      } else {
        servo2.write(90);   // Back to center
      }
    }
  }
}