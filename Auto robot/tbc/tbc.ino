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
float offset[8] = {1, 0.99, 1.04, 0.89, 0.77, 0.86, 0.83, 0.89};
int cross_value = 570;
int value90 = 360;

//Motor_driver_Pin
int en2 = 5; //red jumper wire
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
float multipier = 1.5;
float multipier_launch = (7/5);
float multipier_slow = 1.0;


//IR_Pin
int IR_left = 11; //blue jumper wire
int IR_right = 10; //blue jumper wire

//IR_Variables
int left_IR = 0;
int right_IR = 0;

// Ultrasonic_Pin
int trig1 = 13; //blue jumper wire (front)
int echo1 = 12; //green jumper wire(front)
int trig++2 = 49; //yellow jumper wire(back)
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
int led1 = 53 ;       // green jumper wire // i2c  // Orange LED
//Position pinout
int TZ1_pin = 38;
int TZ2_pin = 41;
int TZ3_pin = 40;
int reload_pin = 37;
int receive_pin = 36;
//Indication led
int reload_led = 22;
int launch_led = 24;
int detect_led = 26;


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
  Wire.setTimeout(100);
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
  pinMode(TZ1_pin,OUTPUT);
  pinMode(TZ2_pin,OUTPUT);
  pinMode(TZ3_pin,OUTPUT);
  pinMode(reload_pin,OUTPUT);
  pinMode(receieved_pin,INPUT);
  pinMode(led1, OUTPUT);
}

void loop() {
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  if (Wire.available()) {
    digitalWrite(led1, HIGH);       //Check for i2c
  } else {
    digitalWrite(led1, LOW);
  }
  delay(50);
  Serial.print(ultrasonic1.ping_cm());
  delay(50);
  Serial.print("    ");
  Serial.println(ultrasonic2.ping_cm());
//  delay(50);
 // Serial.println(ultrasonic2.ping_cm());
  //StartZone
  //startzone();

  // TZ1
  //goTZ1();
  //TZ1();

  //TZ2
  //goTZ2();
  //TZ2();

  //TZ3
  //TZ3();
 // line_follow_reverse();
  //read_sunfounder();
  //Serial.println(WA);
  //Serial.println("asdad");
  //go_launchzone();               //Allignment of launching zone
  //go_reverse();
  
}

void startzone()
{
  turnRight90();                    //First right turn on a 90 junction
  go_straight();
}

void goTZ1()
{
  turnLeft90();                   // Enter TZ1
  delay(3000);
  go_straightTZ1();
}


void TZ1()
{
  digitalWrite(led16, LOW);
  digitalWrite(led17, LOW);
  go_launchzone();               //Allignment of launching zone
  go_reverse();
  check_reloadzone();
}

void goTZ2()
{
  go_straight();
  turnRightcrossTZ2();
  turnLeft90TZ2();
  delay(3000);
}


void TZ2()
{
  digitalWrite(led16, LOW);
  digitalWrite(led17, LOW);
  check_launchzoneTZ2();
  go_reverseTZ2();
  check_reloadzoneTZ2();
}

void TZ3()
{
  while (1) {
    go_straightTZ3();
    check_launchzoneTZ3();
    go_reverseTZ3();
    check_reloadzoneTZ3();
  }
}

void turnRight90() {
  digitalWrite(led2, HIGH);
  while (1) {
    line_follow(forward);                 //Line follow in forward direction
    //debug line
    //Serial.println(sumvalue);
    //Serial.print(rightMost);
    //Serial.print("  ");
    //Serial.print(right2Most);
    //Serial.print("  ");
    //Serial.println(right3Most);

    //Check condition
    if (sumvalue > 530 && sumvalue < 570) {
      digitalWrite(led2, LOW);
      digitalWrite(led3, HIGH);
      //debug line
      //Serial.print("I see a right turn");
      smallreverse();         //Re-adjust the robot for perfect turning
      turnRightcross();       //Turn right by 90
      digitalWrite(led3, LOW);
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
  steps = 0;
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

void go_straight() {
  steps = 0;
  while (1) {
    line_follow(forward);
    //Serial.println("go straight");
    if (steps >= 50) {
      //Serial.println("Start check 2nd cross");
      steps = 0;
      break;
    }
  }
}

void turnLeft90() {
  digitalWrite(led4, HIGH);
  while (1) {
    line_follow(forward);
    //Serial.println(sumvalue);
    if (sumvalue >= cross_value ) {
      steps = 0;
      smallreverse1();
      turnLeftcross();
      digitalWrite(led4, LOW);
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
    if (steps > 15) {
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
  TurnCross();
}

void TurnCross() {
  //debug line
  //Serial.print("Turn 90");
  while (1) {
    if (steps >= steps_cross) {        //Turn 90
      motorStop();                  //Motor Stops after turning 90
      steps = 0;                    //Reset Encoder value
      break;
    }
  }
}

void go_straightTZ1() {
  steps = 0;
  while (1) {
    line_follow(forward);
    //Serial.println("go straight");
    if (steps >= 150) {
      //Serial.println("Start check 2nd cross");
      steps = 0;
      break;
    }
  }
}


void go_launchzone() {
  digitalWrite(led5, HIGH);
  while (1) {
    line_follow_launch(forward);
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
          digitalWrite(led5, LOW);
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
          digitalWrite(led5, LOW);
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
  digitalWrite(led6, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  steps = 0;
  while (1) {
    //    read_sunfounder();
    line_follow(reverse);
    //Serial.println("go straight");
    if (steps >= 300) {
      digitalWrite(led6, LOW);
      break;
    }
  }
}

void check_reloadzone() {
  digitalWrite(led7, HIGH);
  while (1) {
    line_follow(reverse);
    if (sumvalue >= cross_value) {
      digitalWrite(led7, LOW);
      motorStop();
      delay(4000);
      while (1)
      {
        if (close_flag == 0) {
          check_ultrasonic1();
        }
        else if (close_flag == 2)
        {
          close_flag = 0;
          break;
        }
        else if (close_flag == 3)
        {
          close_flag = 0;
          TZ1();
          break;
        }
        else {
          check_ultrasonic2();              //if manual bot is there, go TZ1
        }
      }
      break;
    }
  }
}

void check_ultrasonic1() {
  delay(50);
  sonar1 = ultrasonic1.ping_cm();
  //Serial.print("ist ultrasonic : ");
  //Serial.println(sonar1);
  if (sonar1 <= manual_dist && sonar1 > 1) {
    //Serial.println("lst detected");
    close_flag = 1;
    digitalWrite(led16, HIGH);
  }
  else
  {
    close_flag = 2;           // He no see manual robot
  }
}

void check_ultrasonic2 () {
  delay(50);
  sonar2 = ultrasonic2.ping_cm();
  //Serial.print("2nd ultrasonic : ");
  //Serial.println(sonar2);
  if (sonar2 <= manual_dist && sonar2 > 0) {
    digitalWrite(led17, HIGH);
    close_flag = 3;
  }
}


void turnRightcrossTZ2() {
  digitalWrite(led8, HIGH);
  //debug line
  //Serial.print("Turning Right");
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);     //Turning with encoder
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 75);
  digitalWrite(dir1, LOW);
  analogWrite(en1, 75);
  Turn90();
  digitalWrite(led8, LOW);
}

void turnLeft90TZ2() {
  digitalWrite(led9, HIGH);
  while (1) {
    line_follow(forward);
    //Serial.println(sumvalue);
    if (sumvalue >= cross_value ) {
      steps = 0;
      smallreverse();
      turnLeftcross();
      digitalWrite(led9, LOW);
      break;
    }
  }
}

void check_launchzoneTZ2() {
  digitalWrite(led10, HIGH);
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
          digitalWrite(led10, LOW);
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
          digitalWrite(led10, LOW);
          delay(5000);
          break;
        }
      }
      break;
    }
  }
}

void go_reverseTZ2() {
  digitalWrite(led11, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  steps = 0;
  while (1) {
    //    read_sunfounder();
    line_follow(reverse);
    //Serial.println("go straight");
    if (steps >= 1000) {
      digitalWrite(led11, LOW);
      break;
    }
  }
}

void check_reloadzoneTZ2() {
  digitalWrite(led12, HIGH);
  while (1) {
    line_follow(reverse);
    if (sumvalue >= cross_value) {
      digitalWrite(led12, LOW);
      motorStop();
      delay(4000);
      while (1)
      {
        if (close_flag == 0) {
          check_ultrasonic1TZ2();
        }
        else if (close_flag == 2)
        {
          close_flag = 0;
          break;
        }
        else if (close_flag == 3)
        {
          close_flag = 0;
          TZ2();
          break;
        }
        else {
          check_ultrasonic2();              //if manual bot is there, go TZ2
        }
      }
      break;
    }
  }
}

void check_ultrasonic1TZ2() {
  delay(50);
  sonar1 = ultrasonic1.ping_cm();
  //Serial.print("ist ultrasonic : ");
  //Serial.println(sonar1);
  if (sonar1 <= manual_dist && sonar1 > 1) {
    //Serial.println("lst detected");
    close_flag = 1;
    digitalWrite(led16, HIGH);
  }
  else
  {
    close_flag = 2;           // He no see manual robot
  }
}

void check_ultrasonic2TZ2() {
  delay(50);
  sonar2 = ultrasonic2.ping_cm();
  //Serial.print("2nd ultrasonic : ");
  //Serial.println(sonar2);
  if (sonar2 <= manual_dist && sonar2 > 0) {
    //Serial.println("2nd detected");
    digitalWrite(led17, HIGH);
    close_flag = 3;
  }
}

void go_straightTZ3() {
  steps = 0;
  digitalWrite(led13, HIGH);
  while (1) {
    line_follow(forward);
    //Serial.println("go straight");
    if (steps >= 1500) {
      //Serial.println("Start check 2nd cross");
      steps = 0;
      break;
    }
  }
}

void check_launchzoneTZ3() {
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
          digitalWrite(led13, LOW);
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
          digitalWrite(led13, LOW);
          delay(5000);
          break;
        }
      }
      break;
    }
  }
}

void go_reverseTZ3() {
  digitalWrite(led14, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  steps = 0;
  while (1) {
    //read_sunfounder();
    line_follow(reverse);
    //Serial.println("go straight");
    if (steps >= 2000) {
      digitalWrite(led14, LOW);
      break;
    }
  }
}

void check_reloadzoneTZ3() {
  digitalWrite(led2, HIGH);
  while (1) {
    line_follow(reverse);
    if (sumvalue >= cross_value) {
      digitalWrite(led2, LOW);
      motorStop();
      delay(2000);
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
  if (WA >= 3.3) {
    speedl = (normal_speed * multipier);
    speedr = (normal_speed * multipier) - (10* multipier);
    if (WA >= 4) {
      speedl += (17.5 * multipier);
      if (WA >= 4.9) {
        speedl += (10* multipier);
        if (data[0]*offset[0] >= 70.0) {
          speedl += (15* multipier);
          speedr = (25* multipier);
        }
      }
    }
  } else if (WA <= 2.2) {
    speedl = (normal_speed * multipier)- (10* multipier);
    speedr = (normal_speed* multipier);
    if (WA <= 1.3) {
      speedr += (17.5* multipier);
      if (WA <= -0.5) {
        speedr += (20* multipier);
        if (data[14]*offset[7] >= 70.0) {
          speedr += (15* multipier);
          speedl = (25* multipier);
        }
      }
    }
  } else {
    speedr = (normal_speed* multipier);
    speedl = (normal_speed* multipier);
  }
  motorLeft(speedl, dir);
  motorRight(speedr, dir);
}

void line_follow_launch(int dir) {
  read_sunfounder();
  if (WA >= 3.3) {
    speedl = (normal_speed * multipier_launch);
    speedr = (normal_speed * multipier_launch) - (10* multipier_launch);
    if (WA >= 4) {
      speedl += (17.5 * multipier_launch);
      if (WA >= 4.9) {
        speedl += (10* multipier_launch);
        if (data[0]*offset[0] >= 70.0) {
          speedl += (15* multipier_launch);
          speedr = (25* multipier_launch);
        }
      }
    }
  } else if (WA <= 2.2) {
    speedl = (normal_speed * multipier_launch)- (10* multipier_launch);
    speedr = (normal_speed* multipier_launch);
    if (WA <= 1.3) {
      speedr += (17.5* multipier_launch);
      if (WA <= -0.5) {
        speedr += (20* multipier_launch);
        if (data[14]*offset[7] >= 70.0) {
          speedr += (15* multipier_launch);
          speedl = (25* multipier_launch);
        }
      }
    }
  } else {
    speedr = (normal_speed* multipier_launch);
    speedl = (normal_speed* multipier_launch);
  }
  motorLeft(speedl, dir);
  motorRight(speedr, dir);
}

void line_follow_reverse() {
  read_sunfounder();
  if (WA >= 3.3) {
    speedl = (normal_speed * multipier_slow);
    speedr = (normal_speed * multipier_slow) - (10* multipier_slow);
    if (WA >= 4) {
      speedl += (19 * multipier_slow);
      if (WA >= 4.9) {
        speedl += (21* multipier_slow);
        if (data[0]*offset[0] >= 70.0) {
          speedl += (15* multipier_slow);
          speedr = (25* multipier_slow);
        }
      }
    }
  } else if (WA <= 2.2) {
    speedl = (normal_speed * multipier_slow)- (10* multipier_slow);
    speedr = (normal_speed* multipier_slow);
    if (WA <= 1.3) {
      speedr += (19* multipier_slow);
      if (WA <= -0.5) {
        speedr += (21* multipier_slow);
        if (data[14]*offset[7] >= 70.0) {
          speedr += (15* multipier_slow);
          speedl = (25* multipier_slow);
        }
      }
    }
  } else {
    speedr = (normal_speed* multipier_slow);
    speedl = (normal_speed* multipier_slow);
  }
  motorLeft(speedl, reverse);
  motorRight(speedr, reverse);
}


float weightedAverage() {
  //   Serial.println("bbbbb");
  read_sunfounder();

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
  sumvalueweight = ((data[0] * (-42) * offset[0]) + (data[2] * (-30) * offset[1]) + (data[4] * (-18) * offset[2]) + (data[6] * (-6) * offset[3]) + (data[8] * 6 * offset[4]) + (data[10] * 18 * offset[5]) + (data[12] * 30 * offset[6]) + (data[14] * 42 * offset[7]));
  sumvalue = ((data[0] * offset[0]) + (data[2] * offset[1]) + (data[4] * offset[2]) + (data[6] * offset[3]) + (data[8] * offset[4]) + (data[10] * offset[5]) + (data[12] * offset[6]) + (data[14] * offset[7]));
  d = (sumvalueweight) / (sumvalue);
  rightMost = (data[14] * offset[7]);
  leftMost = (data[0] * offset[1]);
  right3Most = (data[12] * offset[6]);
  right2Most = (data[10] * offset[5]);
  //Serial.print("sumvalue ==   ");
  //Serial.println(sumvalue);
  //delay(500);
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
    //Serial.println(WA);
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





