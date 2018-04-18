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
float offset[8] = {1, 0.95, 0.99, 0.83, 0.71, 0.8, 0.74, 0.78};
int cross_value = 560;
int value90 = 360;

//Motor_driver_Pin
int en2 = 5; //red jumper wire
int dir2 = 7; //white jumper wire
int en1 = 6; //yellow jumper wire
int dir1 = 8; //green jumper wire

//Motor_Pin
int encoder1 = 3; //yellow jumper wire
//int encoder2 = 3;

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
float multipier = (7 / 5);
float multipier_launch = (7 / 5);
float multipier_slow = (7 / 5);


//IR_Pin
int IR_left = 11; //blue jumper wire
int IR_right = 10; //blue jumper wire

//IR_Variables
int left_IR = 0;
int right_IR = 0;

// Ultrasonic_Pin
int trig1 = 13; //blue jumper wire (front)
int echo1 = 12; //green jumper wire(front)
int trig2 = 49; //yellow jumper wire(back)
int echo2 = 48; //red jumper wire(back)

//Ultrasonic_Variables
int max_dist = 200;
double sonar1 = 0;
double sonar2 = 0;
double manual_dist = 15.0;
NewPing ultrasonic1(trig1, echo1, max_dist); // NewPing setup of pins and maximum distance.
NewPing ultrasonic2(trig2, echo2, max_dist); // NewPing setup of pins and maximum distance.

//for ADS sensor
int ADS = A0;
double ADS_value;
int k = 13 * 200;
long InitialRightSensor, CurrentRightSensor, InitialTime;

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
  pinMode(TZ1_pin, OUTPUT);
  pinMode(TZ2_pin, OUTPUT);
  pinMode(TZ3_pin, OUTPUT);
  pinMode(reload_pin, OUTPUT);
  pinMode(receive_pin, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(ADS, INPUT);
}

void loop() {
  Wire.requestFrom(9, 16); //request 16 bytes from slave device #9
  if (Wire.available()) {
    digitalWrite(led1, HIGH);       //Check for i2c
  } else {
    digitalWrite(led1, LOW);
  }
  //delay(50);
  //Serial.println(ultrasonic2.ping_cm());
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

  //go back to TZ2 after throwing 5 golden balls

  //line_follow_reverse();
  //read_sunfounder();
  //Serial.println(WA);
  //Serial.println(sumvalue);
  //go_launchzone();               //Allignment of launching zone
  //go_reverse();
  //Serial.println("In the loop");
  //check_reloadzone();
  //check_ADS();
  //line_follow(forward);
  //grab_ballball();
}

void startzone()
{
  turnRight90_slow();                    //First right turn on a 90 junction
  //turnRight90_slow();
  go_straight();
}

void goTZ1()
{
  turnLeft90();                   // Enter TZ1
}


void TZ1()
{
  grab_ballball();
  go_launchzone();               //Allignment of launching zone
  go_reverse();
  check_reloadzone();
}

void grab_ballball() {
  InitialRightSensor = k / analogRead(ADS);
  for (int i = 0; i < 4; i++)
  {
    CurrentRightSensor = k / analogRead(ADS);
    if (CurrentRightSensor == 1300 || InitialRightSensor == 1300) {
      i = 0;
    } else if (InitialRightSensor == CurrentRightSensor && CurrentRightSensor != 1300) {
      InitialTime = millis();
      while (millis() - InitialTime < 500);
      Serial.println ("Aligned for now");
    } else {
      i = 0;
      Serial.println ("Misaligned");
    }
    //Serial.print(InitialRightSensor);
    //Serial.print ("         ");
    //Serial.println (CurrentRightSensor);
    InitialRightSensor = CurrentRightSensor;
    delay(50);
  }
  //Serial.println("Aligned");
  //Serial.println("grabgrab");
  digitalWrite(reload_pin, HIGH);
  while (1) {
    digitalWrite(reload_pin, LOW);
    int receive = digitalRead(receive_pin);
    if (receive == 1) {
      break;
    }
    break;
  }
}

void goTZ2()
{
  smallreverse();
  turnRightcrossTZ2();
  go_straight();
  turnLeft90TZ2();
}


void TZ2()
{
  grab_ballball();
  check_launchzoneTZ2();
  go_reverseTZ2();
  check_reloadzoneTZ2();
}

void TZ3()
{
  grab_ballball_TZ3();
  go_straightTZ3();
  check_launchzoneTZ3();
  go_reverseTZ3();
  check_reloadzoneTZ3();
}
while (1);
}

void grab_ballball_TZ3() {
  int throw_count = 0;
  for (throw_count = 0; throw_count > 5; throw_count++) {
    InitialRightSensor = k / analogRead(ADS);
    for (int i = 0; i < 4; i++)
    {
      CurrentRightSensor = k / analogRead(ADS);
      if (CurrentRightSensor == 1300 || InitialRightSensor == 1300) {
        i = 0;
      } else if (InitialRightSensor == CurrentRightSensor && CurrentRightSensor != 1300) {
        InitialTime = millis();
        while (millis() - InitialTime < 500);
        Serial.println ("Aligned for now");
      } else {
        i = 0;
        Serial.println ("Misaligned");
      }
      //Serial.print(InitialRightSensor);
      //Serial.print ("         ");
      //Serial.println (CurrentRightSensor);
      InitialRightSensor = CurrentRightSensor;
      delay(50);
    }
    //Serial.println("Aligned");
    //Serial.println("grabgrab");
    digitalWrite(reload_pin, HIGH);
    delay(3000);
    while (1) {
      digitalWrite(reload_pin, LOW);
      int receive = digitalRead(receive_pin);
      if (receive == 1) {
        break;
      }
      break;
    }
  }
}

void turnRight90() {
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
    if (sumvalue > cross_value) {
      //debug line
      //Serial.print("I see a right turn");
      smallreverse();         //Re-adjust the robot for perfect turning
      turnRightcross();       //Turn right by 90
      break;
    }
  }
}

void turnRight90_slow() {
  while (1) {
    line_follow_launch(forward);                 //Line follow in forward direction
    //debug line
    //Serial.println(sumvalue);
    //Serial.print(rightMost);
    //Serial.print("  ");
    //Serial.print(right2Most);
    //Serial.print("  ");
    //Serial.println(right3Most);

    //Check condition
    if (sumvalue > cross_value) {
      //debug line
      //Serial.print("I see a right turn");
      smallreverse();         //Re-adjust the robot for perfect turning
      turnRightcross();       //Turn right by 90
      break;
    }
  }
}

void smallreverse() {
  //debug line
  //Serial.print("I need to reverse a little bit for perfect turn");
  steps = 0;
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
    if (steps >= 100) {
      //Serial.println("Start check 2nd cross");
      steps = 0;
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
  steps = 0;
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

////////////////////////////////////// TZ1 //////////////////////////

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
          digitalWrite(TZ1_pin, HIGH);    // Launching
          while (1) {
            delay(5000);
            digitalWrite(TZ1_pin, LOW);
            int receive = digitalRead(receive_pin);
            if (receive == 1) {
              break;
            }
            break;
          }
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
          digitalWrite(TZ1_pin, HIGH);  // Launching
          while (1) {
            delay(5000);
            digitalWrite(TZ1_pin, LOW);
            int receive = digitalRead(receive_pin);
            if (receive == 1) {
              break;
            }
            break;
          }
          break;
        }
      }
      break;
    }
  }
}


void go_reverse() {
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  steps = 0;
  while (1) {
    //    read_sunfounder();
    line_follow_reverse();
    //Serial.println("go straight");
    if (steps >= 300) {
      break;
    }
  }
}

void check_reloadzone() {
  while (1) {
    line_follow_reverse();
    if (sumvalue >= cross_value) {
      motorStop();
      //Serial.println("stop at juction");
      unsigned long t1 = millis();
      unsigned long t2 = millis();
      while ((t2 - t1) < 4000) {
        t2 = millis();
        //Serial.print(t1);
        //Serial.print("  ");
        //Serial.println(t2);
        //check_ultrasonic2();
        check_ADS();
        motorStop();
        //Serial.println("checking ultrasonic");
      }
      if (close_flag == 3) {
        //Serial.println("MR detected");
        motorStop();
        delay(4000);
        TZ1();
        //go_launchzone();
        //Serial.println("Go TZ1");
        break;
      }
      //Serial.println("Go TZ2");
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
  }
  else
  {
    close_flag = 2;           // He no see manual robot
  }
}

void check_ultrasonic2() {
  delay(50);
  sonar2 = ultrasonic2.ping_cm();
  Serial.print("2nd ultrasonic : ");
  Serial.println(sonar2);
  if (sonar2 <= manual_dist && sonar2 > 0) {
    close_flag = 3;
    Serial.println("flag is 3!!!!");
  }
}

void check_ADS() {
  int counterADS = 0;
  ADS_value = k / analogRead(ADS);
  Serial.println(ADS_value);
  if (ADS_value <= manual_dist && ADS_value > 0) {
    counterADS++;
    if (counterADS >= 5) {
      close_flag = 3;
      //Serial.println("flag is 3!");
    }
  }
}

/////////////////////////////////////  TZ2 ///////////////////////////////////////////

void turnRightcrossTZ2() {
  //debug line
  //Serial.print("Turning Right");
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);     //Turning with encoder
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 75);
  digitalWrite(dir1, LOW);
  analogWrite(en1, 75);
  Turn90();
}

void turnLeft90TZ2() {
  while (1) {
    line_follow(forward);
    //Serial.println(sumvalue);
    if (sumvalue >= cross_value ) {
      steps = 0;
      smallreverse();
      turnLeftcross();
      break;
    }
  }
}

void check_launchzoneTZ2() {
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
          digitalWrite(TZ2_pin, HIGH);    // Launching
          while (1) {
            delay(5000);
            digitalWrite(TZ2_pin, LOW);
            int receive = digitalRead(receive_pin);
            //if (receive == 1) {
            // break;
            //}
            break;
          }
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
          digitalWrite(TZ2_pin, HIGH);    // Launching
          while (1) {
            delay(5000);
            digitalWrite(TZ2_pin, LOW);
            int receive = digitalRead(receive_pin);
            //if (receive == 1) {
            // break;
            //}
            break;
          }
          break;
        }
      }
      break;
    }
  }
}

void go_reverseTZ2() {
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  steps = 0;
  while (1) {
    //    read_sunfounder();
    line_follow(reverse);
    //Serial.println("go straight");
    if (steps >= 1000) {
      break;
    }
  }
}

void check_reloadzoneTZ2() {
  while (1) {
    line_follow_reverse();
    if (sumvalue >= cross_value) {
      motorStop();
      //Serial.println("stop at juction");
      unsigned long t1 = millis();
      unsigned long t2 = millis();
      while ((t2 - t1) < 4000) {
        t2 = millis();
        //Serial.print(t1);
        //Serial.print("  ");
        //Serial.println(t2);
        //check_ultrasonic2();
        check_ADS();
        motorStop();
        //Serial.println("checking ultrasonic");
      }
      if (close_flag == 3) {
        //Serial.println("MR detected");
        motorStop();
        delay(4000);
        TZ2();
        //go_launchzone();
        //Serial.println("Go TZ1");
        break;
      }
      //Serial.println("Go TZ2");
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
    close_flag = 3;
  }
}

/////////////////////////////////////  TZ3 ///////////////////////////////////////////
void go_straightTZ3() {
  steps = 0;
  while (1) {
    line_follow_launch(forward);
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
          digitalWrite(TZ3_pin, HIGH);    //Launching
          while (1) {
            delay(5000);
            digitalWrite(TZ3_pin, LOW);
            int receive = digitalRead(receive_pin);
            if (receive == 1) {
              break;
            }
            break;
          }
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
          digitalWrite(TZ3_pin, HIGH);     // Launching
          while (1) {
            delay(5000);
            digitalWrite(TZ3_pin, LOW);   // Launching
            int receive = digitalRead(receive_pin);
            //if (receive == 1) {
            // break;
            //}
            break;
          }
          break;
        }
      }
      break;
    }
  }
}

void go_reverseTZ3() {
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  steps = 0;
  while (1) {
    //read_sunfounder();
    line_follow_reverse();
    //Serial.println("go straight");
    if (steps >= 2000) {
      break;
    }
  }
}

void check_reloadzoneTZ3() {
  while (1) {
    line_follow_reverse();
    if (sumvalue >= cross_value) {
      motorStop();
      delay(2000);
      break;
    }
  }
}

////////////////////////////////////////////// MOTOR FUNCTION ///////////////////////////
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

////////////////// LINE FOLLOW FUNCTION //////////////////////////////////
void line_follow(int dir) {
  read_sunfounder();
  speedl = (normal_speed * multipier);
  speedr = (normal_speed * multipier);
  if (WA >= 0.4) {
    speedl = (normal_speed * multipier);
    speedr = (normal_speed * multipier) - (10 * multipier);
    if (WA >= 2.4) {
      speedl += (17.5 * multipier);
      if (WA >= 3.81) {
        speedl += (10 * multipier);
        if (data[0]*offset[0] >= 70.0) {
          speedl += (15 * multipier);
          speedr = (25 * multipier);
        }
      }
    }
  } else if (WA <= -0.6) {
    speedl = (normal_speed * multipier) - (10 * multipier);
    speedr = (normal_speed * multipier);
    if (WA <= -1.58) {
      speedr += (17.5 * multipier);
      if (WA <= -2.8 ) {
        speedr += (20 * multipier);
        if (data[14]*offset[7] >= 70.0) {
          speedr += (15 * multipier);
          speedl = (25 * multipier);
        }
      }
    }
  } else {
    speedr = (normal_speed * multipier);
    speedl = (normal_speed * multipier);
  }
  motorLeft(speedl, dir);
  motorRight(speedr, dir);
}

void line_follow_launch(int dir) {
  read_sunfounder();
  speedl = (normal_speed * multipier_launch);
  speedr = (normal_speed * multipier_launch);
  if (WA >= 0.4) {
    speedl = (normal_speed * multipier_launch);
    speedr = (normal_speed * multipier_launch) - (10 * multipier_launch);
    if (WA >= 2.4) {
      speedl += (17.5 * multipier_launch);
      if (WA >= 3.81) {
        speedl += (10 * multipier_launch);
        if (data[0]*offset[0] >= 70.0) {
          speedl += (15 * multipier_launch);
          speedr = (25 * multipier_launch);
        }
      }
    }
  } else if (WA <= -0.6) {
    speedl = (normal_speed * multipier_launch) - (10 * multipier_launch);
    speedr = (normal_speed * multipier_launch);
    if (WA <= -1.58) {
      speedr += (17.5 * multipier_launch);
      if (WA <= -2.8) {
        speedr += (20 * multipier_launch);
        if (data[14]*offset[7] >= 70.0) {
          speedr += (15 * multipier_launch);
          speedl = (25 * multipier_launch);
        }
      }
    }
  } else {
    speedr = (normal_speed * multipier_launch);
    speedl = (normal_speed * multipier_launch);
  }
  motorLeft(speedl, dir);
  motorRight(speedr, dir);
}

void line_follow_reverse() {
  speedl = (normal_speed * multipier_slow);
  speedr = (normal_speed * multipier_slow);
  read_sunfounder();
  if (WA >= 0.4) {
    speedl = (normal_speed * multipier_slow);
    speedr = (normal_speed * multipier_slow) - (10 * multipier_slow);
    if (WA >= 2.4) {
      speedl += (17.5 * multipier_slow);
      if (WA >= 3.81) {
        speedl += (20 * multipier_slow);
        if (data[0]*offset[0] >= 70.0) {
          speedl += (15 * multipier_slow);
          speedr = (25 * multipier_slow);
        }
      }
    }
  } else if (WA <= -0.6) {
    speedl = (normal_speed * multipier_slow) - (10 * multipier_slow);
    speedr = (normal_speed * multipier_slow);
    if (WA <= -1.58) {
      speedr += (17.5 * multipier_slow);
      if (WA <= -2.8) {
        speedr += (20 * multipier_slow);
        if (data[14]*offset[7] >= 70.0) {
          speedr += (15 * multipier_slow);
          speedl = (25 * multipier_slow);
        }
      }
    }
  } else {
    speedr = (normal_speed * multipier_slow);
    speedl = (normal_speed * multipier_slow);
  }
  motorLeft(speedl, reverse);
  motorRight(speedr, reverse);
}

/////////////////////////// SUNFOUNDER FUNCTION ///////////////////////
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
  {
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
    Serial.println("WA:");
    Serial.print(WA);
    Serial.print(" sumValue");
    Serial.println(sumvalue);
    Serial.println(" rightMost:");
    Serial.println(rightMost);
  }
}

///////////////////////////// FOR ENCODER /////////////////////////
void cal() {
  steps++;
  //Serial.println(steps);
  if (steps >= 10000) {
    steps = 0;
  }
}


void read_IR() {
  left_IR = digitalRead(IR_left);
  right_IR = digitalRead(IR_right);
}


