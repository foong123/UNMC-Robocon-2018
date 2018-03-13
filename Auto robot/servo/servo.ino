#include <Servo.h>       //Servo library
 
Servo servo_test;        //initialize a servo object for the connected servo  
                
int angle = 70;    
int angle_open = 100; 
void setup() 
{ 
  servo_test.attach(9);      // attach the signal pin of servo to pin9 of arduino
  Serial.begin(9600);
} 
  
void loop() 
{ 
  servo_test.write(angle);
  delay(1000);
  if(Serial.read()=='c'){
  if(millis() >= 10000)
  {
    servo_test.write(angle_open);
    delay(100);
    while(1);
  
  }
}
}
