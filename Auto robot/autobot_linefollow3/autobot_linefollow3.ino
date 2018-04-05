#include <Wire.h>
#include <NewPing.h>
#define uchar unsigned char
#define forward 0
#define reverse 1

//Sunfounder_Pin
//sda -- blue jumper wire
//scl -- orange jumper wire

int leftright = 31;
int power1 = 33;
int power2 = 35;
int power3 = 37;

int WA_direction;
int multiply1;
int multiply2;
int multiply3;

int leftspeed;
int rightspeed;

// Sunfounder_Variables
uchar t;
uchar data[16];
float sumvalueweight;
float sumvalue;
float WA;
float d;
float rightMost;
float right2Most;
float right3Most;
float right4Most;
float leftMost;
float left2Most;
float left3Most;
float left4Most;
float offset[8] = {1, 0.93, 0.98, 0.86, 0.76, 0.91, 0.89, 0.96};
int cross_value = 600;

//Motor_driver_Pin
int en2 = 5; //red jumper wire // right
int dir2 = 7; //white jumper wire
int en1 = 6; //yellow jumper wire
int dir1 = 8; //green jumper wire

//Motor_Pin
int encoder1 = 2; //yellow jumper wire
int encoder2 = 3;

//Motor_Variables
int leftMotorBaseSpeed = 70;
int rightMotorBaseSpeed = 70;
int min_speed = -85;
int max_speed = 85;
int steps_90 = 355;         //Turn 90
int steps_cross = 330;
int speedl = 0;
int speedr = 0;
int normal_speed = 70;
volatile int steps = 0;     //Encoder starting values
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;


//IR_Pin
int IR_left = 10; //blue jumper wire
int IR_right = 11; //blue jumper wire

//IR_Variables
int left_IR = 0;
int right_IR = 0;

// Ultrasonic_Pin
int trig1 = 13; //blue jumper wire (front)
int echo1 = 12; //green jumper wire(front)
int trig2 = 49; //yellow jumper wire(back)
int echo2 = 48; //red jumper wire(back)

//Ultrasonic_Variables
int max_dist = 30;
double sonar1 = 0;
double sonar2 = 0;
double manual_dist = 23.0;
NewPing ultrasonic1(trig1, echo1, max_dist); // NewPing setup of pins and maximum distance.
NewPing ultrasonic2(trig2, echo2, max_dist); // NewPing setup of pins and maximum distance.

//for pid
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0;
float error, errorSum, errorOld;
long output;

//debugLine

// I2C
int led1 = 9;       // green jumper wire // i2c  // Orange LED

//StartZone
int led2 = 28;      // Startzone  // Red LED
int led3 = 30;      // Turn90     // Red LED

//TZ1

int led4 = 32;      // TZ1 Left Cross   // Green LED
int led5 = 34;      // Allignment TZ1   // Green LED
int led6 = 36;      // Reverse TZ1 // Green LED
int led7 = 38;      // Check Manual bot // Green LED

//TZ2
int led8 = 39;      // Turn Right Cross // Blue LED
int led9 = 41;      // TZ2 Left Cross // Blue LED
int led10 = 43;      // Allignment TZ2 // Blue LED
int led11 = 45;      // Reverse TZ2 // Blue LED
int led12 = 47;     // Check Manual Bot // Blue LED

// TZ3
int led13 = 24;      // TZ3 go forward + allignment // RED LED
int led14 = 25;      // TZ3 reverse // RED LED
int led15 = 26;      // Check Manual Bot// RED LED

//Flags
int ignore_flag = 0;
int launch_flag = 0;
int reload_flag = 0;
int close_flag = 0;
int stop_flag = 0;

//Unused
int magic_number  = 40;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  error = 0;    // Initialise error variables
  errorSum = 0;
  errorOld = 0;
  //PinMode setup
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(IR_left, INPUT);
  pinMode(IR_right, INPUT);
  //attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  pinMode(en2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);
  pinMode(led7, OUTPUT);
  pinMode(led8, OUTPUT);
  pinMode(led9, OUTPUT);
  pinMode(led10, OUTPUT);
  pinMode(led11, OUTPUT);
  pinMode(led12, OUTPUT);
  pinMode(led13, OUTPUT);
  pinMode(led14, OUTPUT);
  pinMode(led15, OUTPUT);

  //for line follow
  pinMode(leftright, INPUT);
  pinMode(power1, INPUT);
  pinMode(power2, INPUT);
  pinMode(power3, INPUT);
}

void loop() {


  line_follow(forward);




}




void motorLeft(float speed_pwm, int dir) {
  digitalWrite(dir2, dir);
  analogWrite(en2, speed_pwm);
}

void motorRight(float speed_pwm, int dir) {
  digitalWrite(led6, HIGH);
  digitalWrite(dir1, dir);
  analogWrite(en1, speed_pwm);
}

void motorStop() {
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 0);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 0);
  //delay(25);
}

long pid(float lineDist)
{
  errorOld = error;        // Save the old error for differential component
  error = lineDist;  // Calculate the error in position
  errorSum += error;
  //Serial.println(error);
  //delay(500);
  float proportional = error * Kp;  // Calculate the components of the PID
  /*
    float integral = errorSum * Ki;
    float differential = (error - errorOld) * Kd;
    long output = proportional + integral + differential;  // Calculate the result
  */
  return output;
}
/*
  void line_follow(int dir) {
  char code [20] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't'};
  int n = 0;
  while (1) {
    if (Serial.available()) {
      char input_data  = Serial.read();
      for (int i  = 0; i < 20 ; i++) {
        if (code[i] == input_data) {
          n = i;
          break;
        }
      }
      if(n > 10 ){
        int index = n - 10;
        digitalWrite(dir1,dir);
         digitalWrite(dir2,dir);
        analogWrite(en2,normal_speed + index*5);
        analogWrite(en1,normal_speed );
        }else if(n < 10){
          int index = 10 - n;
          digitalWrite(dir1,dir);
         digitalWrite(dir2,dir);
         analogWrite(en2,normal_speed);
        analogWrite(en1,normal_speed + index*5);
          }else{
            digitalWrite(dir1,dir);
         digitalWrite(dir2,dir);
         analogWrite(en2,normal_speed );
        analogWrite(en1,normal_speed );
            }
    }
  }
  }
*/


void line_follow(int dir) {
  //dir2 is right, 1 is backward

  while (1) {
    WA_direction = digitalRead(leftright);
    multiply1 = digitalRead(power1);
    multiply2 = digitalRead(power2);
    multiply3 = digitalRead(power3);
    rightspeed = normal_speed;
    leftspeed = normal_speed;

    if (WA_direction == 1) { //go right
      if (multiply1 == 1) {
        rightspeed = rightspeed + 20
        if (multiply2 == 1) {
          rightspeed = rightspeed + 20
          if (multiply3 == 1) {
            rightspeed = rightspeed + 20
          }
        }
      }
    }

    if (WA_direction == 0) { //go left
      if (multiply1 == 1) {
        leftspeed = leftspeed + 20
        if (multiply2 == 1) {
          leftspeed = leftspeed + 20
          if (multiply3 == 1) {
            leftspeed = leftspeed + 20
          }
        }
      }
    }

    digitalWrite(dir1, dir);
    digitalWrite(dir2, dir);
    analogWrite(en2, rightspeed);
    analogWrite(en1, leftspeed);
  }
}






