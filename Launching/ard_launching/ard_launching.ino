#include <Servo.h>
int encoderA = 2;
int encoderB = 3;
volatile int steps = 0;
double angle_per_steps = 360 / 400;
int motorA = 5;
int motorB = 6;
double release_angle = 135;
double error = 1.0;
Servo myservo;
void setup() {
Serial.begin(9600);
pinMode(encoderA , INPUT_PULLUP);
pinMode(encoderB , INPUT_PULLUP);
pinMode(motorA , OUTPUT);
pinMode(motorB , OUTPUT);
attachInterrupt(digitalPinToInterrupt(encoderA),cal,CHANGE);
attachInterrupt(digitalPinToInterrupt(encoderB),cal,CHANGE);
myservo.attach(9);
myservo.write(70);
Serial.println("Clamped");
while(1){
  if(Serial.read() == 'c'){
      motorstart();
      break;
    }
  
  }
}

void loop() {

}
void motorstart(){
  analogWrite(motorA , 150);
  analogWrite(motorB , 0);
  }

 void motorstop(){
  analogWrite(motorA , 0);
  analogWrite(motorB , 0);
  Serial.println("Motor stopped");
  
  }
void cal(){
  steps ++;
  /* faster version
   if(steps >= release_steps - step_error && steps <= release_steps + step_error ){
    // release servo
   
   }
   * 
   */
  if(steps * angle_per_steps >= release_angle - error && steps * angle_per_steps <= release_angle + error){
    //servo release
    // myservo.write(100);
    //motorstop();
    }
  
  }
