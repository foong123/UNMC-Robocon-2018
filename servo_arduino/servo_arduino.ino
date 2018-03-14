// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

int state; 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);
Serial.begin(9600);
pinMode(8,INPUT);
// attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{ 
  state = digitalRead(8);
  if (state==1){
    Serial.println("hihg")
    myservo.write(70);
  }
  else if (state == 0){
   Serial.println("low")
    myservo.write(100) 
  }
} 
