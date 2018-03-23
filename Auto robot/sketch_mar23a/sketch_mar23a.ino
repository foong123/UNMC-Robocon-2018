#include <Wire.h>
#define uchar unsigned char

//for sunfounder
uchar t;
uchar data[16];
float sumvalueweight;
float sumvalue;
float WA;
float d;

float offset[8] = {1, 1.02, 1.11, 0.99, 0.81, 0.88, 0.82, 0.89};

//for pid
float Kp = 1.0;
float Ki = 0.025;
float Kd = 0;
long output;
float error, errorSum, errorOld;

//for motor movement
int leftMotorBaseSpeed = 35;
int rightMotorBaseSpeed = 35;
int min_speed = -50;
int max_speed = 50;
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;

//pin for motor
int enL = 5;
int dirL = 7;
int enR = 6;
int dirR = 8;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  error = 0;    // Initialise error variables
  errorSum = 0;
  errorOld = 0;

  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(dirR, OUTPUT);
}

void loop() {
  /*
  if (Serial.read() == 'c'){
    motorStop();
  }*/
  WA = weightedAverage();
  output = pid(WA);
  motor_speed(WA);
  //motorLeft(100,0);
  //motorRight(100, 0);
  //turn90();
}

float weightedAverage() {
  Serial.println("bbbbb");
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  while (Wire.available())
  { Serial.println("aaa");
    data[t] = Wire.read();
    if (t < 15) {
      t++;
    }
    else {
      t = 0;
    }
  }

  Serial.print("data[1]:");
  Serial.println(data[0] * offset[0]);
  Serial.print("data[2]:");
  Serial.println(data[2] * offset[1]);
  Serial.print("data[3]:");
  Serial.println(data[4] * offset[2]);
  Serial.print("data[4]:");
  Serial.println(data[6] * offset[3]);
  Serial.print("data[5]:");
  Serial.println(data[8] * offset[4]);
  Serial.print("data[6]:");
  Serial.println(data[10] * offset[5]);
  Serial.print("data[7]:");
  Serial.println(data[12] * offset[6]);
  Serial.print("data[8]:");
  Serial.println(data[14] * offset[7]);
  delay(500);
  sumvalueweight = ((data[0] * (-42) * offset[0]) + (data[2] * (-30) * offset[1]) + (data[4] * (-18) * offset[2]) + (data[6] * (-6) * offset[3]) + (data[8] * 6 * offset[4]) + (data[10] * 18 * offset[5]) + (data[12] * 30 * offset[6]) + (data[14] * 42 * offset[7]));
  sumvalue = ((data[0] * offset[0]) + (data[2] * offset[1]) + (data[4] * offset[2]) + (data[6] * offset[3]) + (data[8] * offset[4]) + (data[10] * offset[5]) + (data[12] * offset[6]) + (data[14] * offset[7]));
  d = (sumvalueweight) / (sumvalue);
  Serial.println(d);
  Serial.print("sumvalue ==   ");
  Serial.println(sumvalue);
  Serial.println(d);
  return d;
}

long pid(float lineDist)
{
  errorOld = error;        // Save the old error for differential component
  error = lineDist;  // Calculate the error in position
  errorSum += error;
  //Serial.println(error);

  float proportional = error * Kp;  // Calculate the components of the PID

  float integral = errorSum * Ki;

  float differential = (error - errorOld) * Kd;

  long output = proportional + integral + differential;  // Calculate the result

  return output;
}

void motor_speed(long output)
{
  leftMotorSpeed = leftMotorBaseSpeed + output;
  rightMotorSpeed = rightMotorBaseSpeed - output;
  Serial.println("aaaaa");

  if (leftMotorSpeed > 23)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, min_speed, max_speed);
    //MOTOR.setSpeedDir1(abs(leftMotorSpeed), DIRR); Forward
    motorLeft(abs(leftMotorSpeed), 1);
    Serial.print(leftMotorSpeed);
    Serial.print("   ");
  }
  else
  {
    leftMotorSpeed = constrain(leftMotorSpeed, min_speed, max_speed );
    //MOTOR.setSpeedDir1(abs(leftMotorSpeed), DIRF); reverse
    motorLeft(abs(leftMotorSpeed), 0);
    Serial.print(leftMotorSpeed);
    Serial.print("   ");
  }

  if (rightMotorSpeed > 23)
  {
    rightMotorSpeed = constrain(rightMotorSpeed, min_speed, max_speed);
    //MOTOR.setSpeedDir2(abs(rightMotorSpeed), DIRF);
    motorRight(abs(rightMotorSpeed), 1);
    Serial.println(rightMotorSpeed);
  }
  else
  {
    rightMotorSpeed = constrain(rightMotorSpeed, min_speed, max_speed);
    //MOTOR.setSpeedDir2(abs(rightMotorSpeed), DIRR);
    motorRight(abs(rightMotorSpeed), 0);
    Serial.println(rightMotorSpeed);
  }

}

void motorLeft(float speed_pwm, int dir) {
  digitalWrite(dirL, dir);
  analogWrite(enL, speed_pwm);
}

void motorRight(float speed_pwm, int dir) {
  digitalWrite(dirR, dir);
  analogWrite(enR, speed_pwm);
}

void turn90() {
  digitalWrite(dirL, LOW);
  analogWrite(enL, 100);
  digitalWrite(dirR, HIGH);
  analogWrite(enR, 100);
  check90();
}

void motorStop() {
  digitalWrite(dirL, HIGH);
  analogWrite(enL, 0);
  digitalWrite(dirR, HIGH);
  analogWrite(enR, 0);
}

void check90() {
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  while (Wire.available())
  {
    data[t] = Wire.read();
    if (t < 15) {
      t++;
    }
    else {
      t = 0;
    }
  }
  float rightMost = (data[14] * offset[7]);
  float leftMost = (data[0] * offset[1]);

  while (1) {
    if (rightMost < 70 && leftMost < 70) {
      break;
    }
  }
  while (1) {
    if (rightMost >= 70 && leftMost >= 70) {
      break;
    }
  }
  motorStop();
}


