#include <Wire.h>
#define uchar unsigned char

//for sunfounder
uchar t;
uchar data[16];float sumvalueweight;
float sumvalue;
float WA;
float d;

float offset[8] = {1, 0.987, 0.97, 0.813, 0.734, 0.821, 0.78, 0.867};

int leftMotorBaseSpeed = 50;
int rightMotorBaseSpeed = 55;
int min_speed = -85;
int max_speed = 85;
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;
int speedl = 0;
int speedr = 0;
int normal_speed = 150;

int en2 = 5;
int dir2 = 7;
int en1 = 6;
int dir1 = 8;

void setup() {
  Wire.begin();
  Serial.begin(115200);

//  error = 0;    // Initialise error variables
  //errorSum = 0;
  //errorOld = 0;

  pinMode(en2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(9, OUTPUT);
}
void loop() {
  WA = weightedAverage();
  Serial.print("weighted average = ");
  Serial.println(WA);
  Wire.requestFrom(9, 16);
  if(Wire.available()){
    digitalWrite(9,HIGH);
  }else{
    digitalWrite(9,LOW);
  }
  //output = pid(WA);
  //motor_speed(output);
   if(WA >= 0.5){
    speedl = normal_speed;
    speedr = normal_speed - 20;
    if(WA >= 2.8){
      speedl += 35;
      if(WA >= 3.5){
        speedl += 20;
        if(data[14]*offset[7] >= 70){
          speedl += 30;
          speedr = 50;
        }
      }
    }
  }else if(WA <= -0.5){
    speedl = normal_speed - 20;
    speedr = normal_speed;
    if(WA <= -2.8){
      speedr += 35;
      if(WA <= -3.5){
        speedr += 20;
        if(data[0]*offset[0] >= 70){
          speedr += 30;
          speedl = 50;
        }
      }
    }
  }else{
    speedr = normal_speed;
    speedl = normal_speed;
  }
  motorLeft(speedl, 0);
  motorRight(speedr, 0);
}



/*
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
*/
void motor_speed(long output)
{
  leftMotorSpeed = leftMotorBaseSpeed + output;
  rightMotorSpeed = rightMotorBaseSpeed - output;
  //Serial.println("aaaaa");

  if (leftMotorSpeed > 40)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, min_speed, max_speed);
    //MOTOR.setSpeedDir1(abs(leftMotorSpeed), dir1); Forward
    motorLeft(abs(leftMotorSpeed), 1);
    //Serial.print(leftMotorSpeed);
    //Serial.print("   ");
  }
  else
  {
    leftMotorSpeed = constrain(leftMotorSpeed, min_speed, max_speed );
    //MOTOR.setSpeedDir1(abs(leftMotorSpeed), DIRF); reverse
    motorLeft(abs(leftMotorSpeed), 0);
    //Serial.print(leftMotorSpeed);
    //Serial.print("   ");
  }

  if (rightMotorSpeed > 40)
  {
    rightMotorSpeed = constrain(rightMotorSpeed, min_speed, max_speed);
    //MOTOR.setSpeedDir2(abs(rightMotorSpeed), DIRF);
    motorRight(abs(rightMotorSpeed), 1);
    //Serial.println(rightMotorSpeed);
  }
  else
  {
    rightMotorSpeed = constrain(rightMotorSpeed, min_speed, max_speed);
    //MOTOR.setSpeedDir2(abs(rightMotorSpeed), dir1);
    motorRight(abs(rightMotorSpeed), 0);
    //Serial.println(rightMotorSpeed);
  }

}

void motorLeft(float speed_pwm, int dir) {
  digitalWrite(dir2, dir);
  analogWrite(en2, speed_pwm);
}

void motorRight(float speed_pwm, int dir) {
  digitalWrite(dir1, dir);
  analogWrite(en1, speed_pwm);
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

void turn90() {
  digitalWrite(dir2, LOW);
  analogWrite(en2, 100);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 100);
  check90();
}
float weightedAverage() {
  //Serial.println("bbbbb");
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  while (Wire.available())
  { //Serial.println("aaa");
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
  /*
  Serial.println(d);
  Serial.print("sumvalue ==   ");
  Serial.println(sumvalue);
  Serial.println(d);
  */
  return d;
}
void motorStop() {
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 0);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 0);
}











