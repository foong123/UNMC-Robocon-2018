#include <Servo.h>
int encoderA = 2;
int encoderB = 3;
volatile int steps = 0;
double angle_per_steps = 360 / 400;
int motorA = 5;
int motorB = 6;
double grab_angle = 85;
double error = 1.0;
Servo myservo;
int grab_flag = 0;
int ready_flag = 0;
double original_angle = 1.0;
int grab_speed = 30;

void setup() {
Serial.begin(9600);
pinMode(encoderA , INPUT_PULLUP);
pinMode(motorA , OUTPUT);
pinMode(motorB , OUTPUT);
attachInterrupt(digitalPinToInterrupt(encoderA),cal,FALLING);
myservo.attach(9);
while(1){
  if(Serial.read() == 'c'){
      motorstart(1);
      break;
    }
  
  }
}

void loop() {
  if(grab_flag == 0){
  angle_check(grab_angle);
  }else if{grab_flag == 1}{
    angle_check(original_angle);
    
    }
}

void angle_check(double angle){
  if(grab_flag == 0){
     motorstart(1);
    }else{
      motorstart(0);
      }
  if(steps * angle_per_steps >= angle - error && steps * angle_per_steps <= angle + error){
    
    motorstop();
    myserov.write();
    grab_flag = 1;
    /*if(grab_flag == 0){
    //grab_flag = 1 ;
    //myservo.write(70);
    delay(500);
    //Serial.println("Clamped");
    //motorstart(0);//reverse
    return;
    }else if(grab_flag == 1){
      //ready_flag = 1;
      //motorstop();
      //Serial.println("Ready to launch");
      while(1);
      }*/
   
    
    
    
    }
  
  }
void motorstart(int dir){
  if(dir == 1){
    analogWrite(motorA , grab_speed);
    analogWrite(motorB , 0);
    }else if (dir == 0){
      analogWrite(motorA , 0);
      analogWrite(motorB , grab_speed);
      
      }
  
  }

 void motorstop(){
  analogWrite(motorA , 0);
  analogWrite(motorB , 0);
  Serial.println("Motor stopped");
  
  }
n
