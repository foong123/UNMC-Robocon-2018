#include <Wire.h>
#include <NewPing.h>
#define uchar unsigned char
#define forward 0
#define reverse 1

//Sunfounder_Pin
//sda -- blue jumper wire
//scl -- orange jumper wire

// Sunfounder_Variables
uchar t;
uchar data[16];
float sumvalueweight;
float sumvalue;
float WA;
float d;
float rightMost;
float leftMost;
float right2Most;
float right3Most;
float offset[8] = {1, 0.93, 0.98, 0.86, 0.76, 0.91, 0.89, 0.96};
int cross_value = 650;

//Motor_driver_Pin
int en2 = 5; //red jumper wire
int dir2 = 7; //white jumper wire
int en1 = 6; //yellow jumper wire
int dir1 = 8; //green jumper wire

//Motor_Pin
int encoder1 = 2; //yellow jumper wire

//Motor_Variables
int leftMotorBaseSpeed = 70;
int rightMotorBaseSpeed = 70;
int min_speed = -85;
int max_speed = 85;
int steps_90 = 355;         //Turn 90
int speedl = 0;
int speedr = 0;
int normal_speed = 50;
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
double manual_dist = 20.0;
NewPing ultrasonic1(trig1, echo1, max_dist); // NewPing setup of pins and maximum distance.
NewPing ultrasonic2(trig2, echo2, max_dist); // NewPing setup of pins and maximum distance.

//for pid
float Kp = 30.0;
float Ki = 0.0;
float Kd = 0;
float error, errorSum, errorOld;
long output;

//debug
int led1 = 9; //green jumper wire

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
}

void loop() {
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  if (Wire.available()) {
    digitalWrite(led1, HIGH);       //Check for i2c
  } else {
    digitalWrite(led1, LOW);
  }

//StartZone
  //turnRight90();                    //First right turn on a 90 junction
  //go_straight();

//TZ1
  turnLeft90();                     //First left turn on Big Cross
  go_straight();
  ignorecross();                    //Ignore small cross
  check_launchzone();               //Allignment of launching zone
  //line_follow(forward);
  go_reverse();
  //check_reloadzone();
  //go_straight();

 //TZ2

 //TZ3


}

void turnRight90() {
  while (1) {
    line_follow(forward);                 //Line follow in forward direction
    //debug line
    //Serial.print(rightMost);
    //Serial.print("  ");
    //Serial.print(right2Most);
    //Serial.print("  ");
    //Serial.println(right3Most);

    //Check condition
    if (rightMost >= 55 && right2Most >= 61 && right3Most >= 65) {
      //debug line
      Serial.print("I see a right turn");
      smallreverse();         //Re-adjust the robot for perfect turning
      turnRightcross();       //Turn right by 90
      break;
    }
  }
}

void smallreverse() {
  //debug line
  //Serial.print("I need to reverse a little bit for perfect turn");
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  while (1) {
    digitalWrite(dir2, HIGH);
    analogWrite(en2, 30);
    digitalWrite(dir1, HIGH);
    analogWrite(en1, 30);
    if (steps > 15) {
      steps = 0;
      break;
    }
  }
}

void turnRightcross() {
  //debug line
  //Serial.print("Turning Right");
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);     //Turning with encoder
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 75);
  digitalWrite(dir1, LOW);
  analogWrite(en1, 75);
  Turn90();
}

void Turn90() {
  //debug line
  //Serial.print("Turn 90");
  while (1) {
    if (steps >= steps_90) {        //Turn 90
      motorStop();                  //Motor Stops after turning 90
      steps = 0;                    //Reset Encoder value
      break;
    }
  }
}

void turnLeft90() {
  while (1) {
    line_follow(forward);
    //Serial.println(sumvalue);
    if (sumvalue >= cross_value ) {
      steps = 0;
      smallreverse1();
      turnLeftcross();
      break;
    }
  }
}

void smallreverse1() {
  //debug line
  //Serial.print("I need to reverse a little bit for perfect turn");
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  while (1) {
    digitalWrite(dir2, HIGH);
    analogWrite(en2, 30);
    digitalWrite(dir1, HIGH);
    analogWrite(en1, 30);
    if (steps > 30) {
      steps = 0;
      break;
    }
  }
}

void turnLeftcross() {
  //debug line
  //Serial.print("Turning Left");
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  digitalWrite(dir2, LOW);
  analogWrite(en2, 75);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 75);
  Turn90();
}

void ignorecross() {
  while (1) {
    //debug line
    //Serial.println("In ignore");
    line_follow(forward);                    //Line follow in forward direction
    read_sunfounder();                       //Reads sunfounder value and correct it
    if (sumvalue >= cross_value) {
      ignore_flag = 1;                       //Raise a flag to ignore small cross
      break;
    }
  }
}

void check_launchzone() {
  while (1) {
    line_follow(forward);
    read_IR();
    //Serial.println(left_IR);
    //Serial.println(right_IR);
    if (left_IR == HIGH ) {
      //Serial.println("left detect");
      analogWrite(en1, 0);
      digitalWrite(dir2, LOW);
      analogWrite(en2, 50);
      while (1) {
        read_IR();
        if (right_IR == HIGH) {
          //Serial.println("right detect");
          analogWrite(en2, 0);
          launch_flag = 1;
          delay(5000);
          break;
        }
      }
      break;
    }
    if (right_IR == HIGH) {
      //Serial.println("right detect");
      analogWrite(en2, 0);
      digitalWrite(dir1, LOW);
      analogWrite(en1, 50);
      while (1) {
        read_IR();
        if (left_IR == HIGH) {
          //Serial.println("left detect");
          analogWrite(en1, 0);
          launch_flag = 1;
          delay(5000);
          break;
        }
      }
      break;
    }
  }
}

void read_IR() {
  left_IR = digitalRead(IR_left);
  right_IR = digitalRead(IR_right);
}

void go_reverse() {
  steps = 0;
  while (1) {
    read_sunfounder();
    line_follow(reverse);
    //Serial.println("go straight");
    if (steps >= 300) {
      //Serial.println("Start check 2nd cross");
      break;
    }
  }
}

void check_reloadzone() {
  while (1) {
    if (close_flag == 0) {
      check_ultrasonic1();
    } else {
      check_ultrasonic2();
    }
    line_follow(reverse);
    read_IR();
    //Serial.println(left_IR);
    //Serial.println(right_IR);
    if (right_IR == HIGH ) {
      //Serial.println("left detect");
      analogWrite(en2, 0);
      digitalWrite(dir1, HIGH);
      analogWrite(en1, 50);
      while (1) {
        read_IR();
        if (left_IR == HIGH) {
          //Serial.println("right detect");
          analogWrite(en1, 0);
          reload_flag = 1;
          delay(5000);
          break;
        }
      }
      break;
    }
    if (left_IR == HIGH) {
      //Serial.println("right detect");
      analogWrite(en1, 0);
      digitalWrite(dir2, HIGH);
      analogWrite(en2, 50);
      while (1) {
        read_IR();
        if (right_IR == HIGH) {
          //Serial.println("left detect");
          analogWrite(en2, 0);
          reload_flag = 1;
          delay(5000);
          break;
        }
      }
      break;
    }
  }
}

void go_straight() {
  steps = 0;
  while (1) {
    line_follow(forward);
    //Serial.println("go straight");
    if (steps >= 100) {
      //Serial.println("Start check 2nd cross");
      steps = 0;
      break;
    }
  }
}

void check_ultrasonic1() {
  delay(50);
  sonar1 = ultrasonic1.ping_cm();
  Serial.print("ist ultrasonic : ");
  Serial.println(sonar1);
  if (sonar1 <= manual_dist && sonar1 > 0) {
    Serial.println("lst detected");
    close_flag = 1;
  }
}

void check_ultrasonic2 () {
  delay(50);
  sonar2 = ultrasonic2.ping_cm();
  Serial.print("2nd ultrasonic : ");
  Serial.println(sonar2);
  if (sonar2 <= manual_dist && sonar2 > 0) {
    Serial.println("2nd detected");
    motorStop();
    while (1);
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

void line_follow(int dir) {
  read_sunfounder();
  if (WA >= 1.4) {
    speedl = normal_speed;
    speedr = normal_speed - 10;
    if (WA >= 3.66) {
      speedl += 17.5;
      if (WA >= 9.63) {
        speedl += 10;
        if (data[14]*offset[7] >= 70) {
          speedl += 15;
          speedr = 25;
        }
      }
    }
  } else if (WA <= 0.6) {
    speedl = normal_speed - 10;
    speedr = normal_speed;
    if (WA <= -4.4) {
      speedr += 17.5;
      if (WA <= -8) {
        speedr += 20;
        if (data[0]*offset[0] >= 70) {
          speedr += 15;
          speedl = 25;
        }
      }
    }
  } else {
    speedr = normal_speed * 2;
    speedl = normal_speed * 2;
  }
  motorLeft(speedl, dir);
  motorRight(speedr, dir);
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
  rightMost = (data[14] * offset[7]);
  leftMost = (data[0] * offset[1]);
  right3Most = (data[12] * offset[6]);
  right2Most = (data[10] * offset[5]);
  // Serial.print("sumvalue ==   ");
  //Serial.println(sumvalue);
  // delay(500);
  return d ;
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
    //Serial.println(sumvalue);
    WA = (sumvalueweight) / (sumvalue);
    rightMost = (data[0] * offset[1]);
    leftMost = (data[14] * offset[7]);
    right3Most = (data[4] * offset[2]);
    right2Most = (data[2] * offset[1]);
  }
}

void cal() {
  steps++;
  //Serial.println(steps);
  if (steps >= 10000) {
    steps = 0;
  }
}





