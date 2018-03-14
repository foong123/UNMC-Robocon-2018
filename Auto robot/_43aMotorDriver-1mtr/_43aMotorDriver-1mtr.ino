#include <Servo.h>

/*
  www.robotedu.lelong.my

  Product : 43A Motor Driver Controller

  //Motor Driver to Arduino & Battery Connection //
  Motor Driver B+ connect to Battery Power Positive
  Motor Driver B- connect to Battery Power Negative/Ground
  Motor Driver M+ connect to DC Motor Output 1
  Motor Driver M- connect to DC Motor Output 2
  Motor Driver Pin VCC connect to +5V on Arduino
  Motor Driver Pin GND connect to GND on Arduino
  Motor Driver Pin R_EN connect to +5V on Arduino //you can solder or connect all the 5V in one pin
  Motor Driver Pin L_EN connect to +5V on Arduino //you can solder or connect all the 5V in one pin
  Motor Driver Pin R_IS connect to A0 on Arduino
  Motor Driver Pin L_IS connect to A1 on Arduino
  Motor Driver Pin RPWM connect to Pin D5 on Arduino
  Motor Driver Pin LPWM connect to Pin D6 on Arduino
*/
/*
  -- Just a basic tutorial for control one motor --

  Before you use this program,make sure at least u know how to upload an program to your Arduino Board.
  And also please follow the connection,please don't reverse polarity for the battery and those 5V and GND wire.
  that will caused short circuit.
*/



int motorA1 = 5; //motor pin RPWM
int motorA2 = 6; //motor pin LPWM
int motorS1 = A0; //for current sensing, if you don't need this feature,you can disable it.
int motorS2 = A1; //for current sensing, if you don't need this feature,you can disable it.

int currentS1 = 0; //to stall the current sensing value for motorS1
int currentS2 = 0; //to stall the current sensing value for motorS2

int i;


Servo myservo;

void setup() {
  Serial.begin(9600); //open serial monitor for current sensing value
  pinMode(motorA1, OUTPUT); //set motor pin as output
  pinMode(motorA2, OUTPUT);
  pinMode(motorS1, INPUT); //set current sensor pin as input
  pinMode(motorS2, INPUT);
  myservo.attach(9);
  myservo.write(70);
}

void loop() {
  //directionA(); //motorA move one direction
  //delay(180000); //delay for 2 sec
  directionB(); //motorA move other direction
  //delay(2000); //delay for 2 sec
  //motorstop();//motor stop
  //delay(2000);//delay for 2 sec
  delay(10000);
  while(1){
    motorstop();
  }
}
void currentsensing () {
  currentS1 = analogRead(motorS1);
  currentS2 = analogRead(motorS2);
  Serial.print(currentS1); //print current value on Serial Monitor
  Serial.print("  |  "); //value is 0 -1023, max is 1023.
  Serial.println(currentS2);
}
void directionA() {
  currentsensing();
  analogWrite(motorA1, 255); //255 is max speed, can change to other value in 0-255 to control speed.
  analogWrite(motorA2, 0);
}
void directionB() {
  currentsensing();
  analogWrite(motorA1, 0);
  analogWrite(motorA2, 255); //255 is max speed, can change to other value in 0-255 to control speed.
}
void motorstop() {
  currentsensing();
  analogWrite(motorA1, 0);
  analogWrite(motorA2, 0);
}

void release() {
  myservo.write(100);
}

