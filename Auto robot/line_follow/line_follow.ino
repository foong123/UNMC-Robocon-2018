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
//float offset[8] = {1, 0.987, 0.97, 0.813, 0.734, 0.821, 0.78, 0.867};
float offset[8] = {1.00 ,1.02, 1.05, 0.90, 0.76 ,0.82, 0.77 ,0.84};

//for pid
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0;
long output;
float error, errorSum, errorOld;

//for motor movement
int leftMotorBaseSpeed = 150;
int rightMotorBaseSpeed = 150;
int min_speed = -85;
int max_speed = 85;
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;
float leftMost;
float rightMost;
int encoder1 = 2;
int steps_90 = 365;

//debug
int led1 = 9;
int magic_number  = 40;
int cross_value = 600;
int ignore_flag = 0;
//pin for motor
int en2 = 5;
int dir2 = 7;
int en1 = 6;
int dir1 = 8;
int speedl = 0;
int speedr = 0;
int normal_speed = 50;
volatile int steps = 0;
void setup() {
  Wire.begin();
  Serial.begin(115200);

  error = 0;    // Initialise error variables
  errorSum = 0;
  errorOld = 0;

  pinMode(encoder1, INPUT_PULLUP);
  pinMode(en2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(led1, OUTPUT);

}

void loop() {

  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  if (Wire.available()) {
    digitalWrite(led1, HIGH);
  } else {
    digitalWrite(led1, LOW);
  }
  //WA = weightedAverage();
  //read_sunfounder();
  line_follow();
  /*
    if(sumvalue >= cross_value){
    attachInterrupt(digitalPinToInterrupt(encoder1),cal,CHANGE);
    turncross();
    go_straight();
    ignorecross();
    Serial.println("After ignore");
    }*/

  /*if(ignore_flag == 1){
     go_straight();
    while(1){
      Serial.println("After flag");
      read_sunfounder();
      line_follow();
      if(sumvalue >= cross_value){
         Serial.println("3rd cross");
        motorStop();
        while(1);
        }
      }
    }*/
}
void go_straight() {
  steps = 0;
  while (1) {
    read_sunfounder();
    line_follow();
    Serial.println("go straight");
    if (steps >= 300) {
      Serial.println("Start check 2nd cross");
      break;
    }

  }

}
//0.9 center
//-4.4 right (1,2)
//-8.02 (0,1)
//(5,6) 3.66
//9.63 (6,7)


void line_follow() {
  read_sunfounder();
  Serial.println(WA);
  if (WA >= 1.4) {
    speedl = normal_speed*2;
    speedr = normal_speed - 10*2;
    if (WA >= 3.66) {
      speedl += 17.5*2;
      if (WA >= 9.63) {
        speedl += 10*2;
        if (data[14]*offset[7] >= 70) {
          speedl += 15*2;
          speedr = 25*2*2;
        }
      }
    }
  } else if (WA <= 0.6) {
    speedl = normal_speed - 10*2;
    speedr = normal_speed;
    if (WA <= -4.4) {
      speedr += 17.5*2;
      if (WA <= -8) {
        speedr += 20*2;
        if (data[0]*offset[0] >= 70) {
          speedr += 15*2;
          speedl = 25*2*2;
        }
      }
    }
  } else {
    speedr = normal_speed*2;
    speedl = normal_speed *2;
  }
  motorLeft(speedl, 0);
  motorRight(speedr, 0);

}
float weightedAverage() {
  //Serial.println("bbbbb");
  
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
  rightMost = (data[14] * offset[7]);
  leftMost = (data[0] * offset[1]);

  // Serial.print("sumvalue ==   ");
  //Serial.println(sumvalue);
  // delay(500);
  return d ;
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
void ignorecross() {
  while (1) {
    Serial.println("In ignore");
    line_follow();
    read_sunfounder();
    if (sumvalue >= cross_value) {
      ignore_flag = 1;
      break;
    }
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

void turncross() {
  digitalWrite(dir2, LOW);
  analogWrite(en2, 75);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 75);
  checkcross();
}

void motorStop() {

  digitalWrite(dir2, HIGH);
  analogWrite(en2, 0);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 0);
  //delay(25);

}

void checkcross() {

  while (1) {

    if (steps >= steps_90) {
      motorStop();
      break;

    }
  }
}



void read_sunfounder() {
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  while (Wire.available())
  { digitalWrite(led1, HIGH);
    data[t] = Wire.read();
    if (t < 15) {
      t++;
    }
    else {
      t = 0;
    }
    sumvalueweight = ((data[0] * (-42) * offset[0]) + (data[2] * (-30) * offset[1]) + (data[4] * (-18) * offset[2]) + (data[6] * (-6) * offset[3]) + (data[8] * 6 * offset[4]) + (data[10] * 18 * offset[5]) + (data[12] * 30 * offset[6]) + (data[14] * 42 * offset[7]));
    sumvalue = ((data[0] * offset[0]) + (data[2] * offset[1]) + (data[4] * offset[2]) + (data[6] * offset[3]) + (data[8] * offset[4]) + (data[10] * offset[5]) + (data[12] * offset[6]) + (data[14] * offset[7]));
    WA = (sumvalueweight) / (sumvalue);
    rightMost = (data[14] * offset[7]);
    leftMost = (data[0] * offset[1]);
    Serial.println(WA);
  }


}


void cal() {
  steps++;


}
