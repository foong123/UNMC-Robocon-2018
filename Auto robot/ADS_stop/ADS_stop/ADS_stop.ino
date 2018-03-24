#include <Wire.h>
#define uchar unsigned char

//for sunfounder
uchar t;
uchar data[16];
float sumvalueweight;
float sumvalue;
float WA;
float d;
float leftmost;
float rightmost;
float offset[8] = {1, 0.987, 0.97, 0.813, 0.734, 0.821, 0.78, 0.867};

//for pid
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0;
long output;
float error, errorSum, errorOld;

//for motor movement
int leftMotorBaseSpeed = 50;
int rightMotorBaseSpeed = 55;
int min_speed = -85;
int max_speed = 85;
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;

//debug
int led1 = 9;
int magic_number  = 40;
int cross_value = 500;

//pin for motor
int en2 = 5;
int dir2 = 7;
int en1 = 6;
int dir1 = 8;

//ADS pin
int ADS0 = A0;
int ADS1 = A1;
int ADS2 = A2;
double input0;
double input1;
double input2;
int k = 200*26;
int distance = 10;
int distance_error = 1;
void setup() {
  Wire.begin();
  Serial.begin(115200);

  error = 0;    // Initialise error variables
  errorSum = 0;
  errorOld = 0;

  pinMode(en2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(led1,OUTPUT);
}

void loop() {
  
  read_sunfounder();
  line_follow();
  
  if(sumvalue >= cross_value){
  turncross();
  }
}
void line_follow(){
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

  void check_manual(){
    input0 = k/analogRead(ADS0);
    input1 = k/analogRead(ADS1);
    input2 = k/analogRead(ADS2);
    if(input0 >= distance - distance_error && input0 <= distance + distance_error ){
       //slowdown
       if(int (input0) >= int(input1) - 1 && int(input0) <=int(input1) + 1){
        if(int (input1) >= int(input2) - 1 && int(input1) <=int(input2) + 1 ){
           if(int (input2) >= int(input0) - 1 && int(input2) <=int(input0) + 1 ){
            motorstop();
            }
          }
        }
      }
    }
float weightedAverage() {
  //Serial.println("bbbbb");
 read_sunfounder();
 /* 
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
  //delay(500);
  */
  sumvalueweight = ((data[0] * (-42) * offset[0]) + (data[2] * (-30) * offset[1]) + (data[4] * (-18) * offset[2]) + (data[6] * (-6) * offset[3]) + (data[8] * 6 * offset[4]) + (data[10] * 18 * offset[5]) + (data[12] * 30 * offset[6]) + (data[14] * 42 * offset[7]));
  sumvalue = ((data[0] * offset[0]) + (data[2] * offset[1]) + (data[4] * offset[2]) + (data[6] * offset[3]) + (data[8] * offset[4]) + (data[10] * offset[5]) + (data[12] * offset[6]) + (data[14] * offset[7]));
  d = (sumvalueweight) / (sumvalue);
 
 // Serial.print("sumvalue ==   ");
  //Serial.println(sumvalue);
  
  return d ;
}





void motorLeft(float speed_pwm, int dir) {
  digitalWrite(dir2, dir);
  analogWrite(en2, speed_pwm);
}

void motorRight(float speed_pwm, int dir) {
  digitalWrite(dir1, dir);
  analogWrite(en1, speed_pwm);
}

void turncross() {
  digitalWrite(dir2, LOW);
  analogWrite(en2, 100);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 100);
  checkcross();
}

void motorStop() {
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 0);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 0);
}

void checkcross() {
  read_sunfounder();
  while (1) {
    read_sunfounder();
    if (rightMost < 70 && leftMost < 70) {
      break;
    }
  }
  while (1) {
    read_sunfounder();
    if (rightMost >= 70 && leftMost >= 70) {
      break;
    }
  }
  motorStop();
}



void read_sunfounder(){
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
rightMost = (data[14] * offset[7]);
leftMost = (data[0] * offset[1]);
  
  }

