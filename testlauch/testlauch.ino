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
int interval = 500;
int starttime = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(9); 
  myservo.write(70);
  delay(1000);// attaches the servo on pin 9 to the servo object
}

void loop() {
  if(Serial.available()){
    if(Serial.read() == 'c'){
      starttime = millis();
      Serial.println("Start");
      while(1){
      if(millis() - starttime > 250){
        myservo.write(100);
        Serial.println("Realease");
        break;
      }
      }
  }
      
    }
}

