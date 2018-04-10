/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(112500);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  Serial.println("Closed");
  myservo.write(10);
  delay(3000);
  Serial.println("Open");
  myservo.write(100);
  delay(3000);
}

