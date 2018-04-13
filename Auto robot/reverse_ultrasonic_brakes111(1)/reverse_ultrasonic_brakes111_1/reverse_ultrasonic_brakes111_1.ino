#include <Wire.h>
#include <NewPing.h>
#define uchar unsigned char
#define forward 0
#define reverse 1
//for sunfounder
//sda -- blue jumper wire
//scl -- orange jumper wire
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
int leftMotorBaseSpeed = 70;
int rightMotorBaseSpeed = 70;
int min_speed = -85;
int max_speed = 85;
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;
float leftMost;
float rightMost;
float right2Most;
float right3Most;
int encoder1 = 2; //yellow jumper wire
int steps_90 = 480;

//debug
int led1 = 9; //green jumper wire
int magic_number  = 40;
int cross_value = 600;
int ignore_flag = 0;
int left_IR = 0;
int right_IR = 0;
int IR_left = 10; //blue jumper wire
int IR_right = 11; //blue jumper wire
int launch_flag = 0;
int reload_flag = 0;
int trig1 = 13; //blue jumper wire (front)
int echo1 = 12; //green jumper wire(front)
int trig2 = 49; //yellow jumper wire(back)
int echo2 = 48; //red jumper wire(back)
int max_dist = 30;
int close_flag = 0;
int stop_flag = 0;
double manual_dist = 20.0;

//pin for motor
int en2 = 5; //red jumper wire
int dir2 = 7; //white jumper wire
int en1 = 6; //yellow jumper wire
int dir1 = 8; //green jumper wire
int speedl = 0;
int speedr = 0;
int normal_speed = 40;
double sonar1 = 0;
double sonar2 = 0;
volatile int steps = 0;
NewPing ultrasonic1(trig1, echo1, max_dist); // NewPing setup of pins and maximum distance.
NewPing ultrasonic2(trig2, echo2, max_dist); // NewPing setup of pins and maximum distance.
int leftright = 31;
int power1 = 33;
int power2 = 35;
int power3 = 37;
int junction = 38;

int WA_direction;
int multiply1;
int multiply2;
int multiply3;
int junction_flag = 0;

int leftspeed;
int rightspeed;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  error = 0;    // Initialise error variables
  errorSum = 0;
  errorOld = 0;

  pinMode(encoder1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  pinMode(en2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(IR_left, INPUT);
  pinMode(IR_right, INPUT);
  pinMode(leftright, INPUT);
  pinMode(power1, INPUT);
  pinMode(power2, INPUT);
  pinMode(power3, INPUT);
  pinMode(junction, INPUT);
}

void loop() {

  line_follow(forward);
  turnLeftcross();
  line_follow(forward);
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

    Serial.println(left_IR);
    Serial.println(right_IR);
    if (right_IR == HIGH ) {
      Serial.println("left detect");
      analogWrite(en2, 0);
      digitalWrite(dir1, HIGH);
      analogWrite(en1, 50);
      while (1) {
        read_IR();
        if (left_IR == HIGH) {
          Serial.println("right detect");
          analogWrite(en1, 0);
          reload_flag = 1;
          delay(5000);
          break;
        }

      }
      break;
    }
    if (left_IR == HIGH) {
      Serial.println("right detect");
      analogWrite(en1, 0);
      digitalWrite(dir2, HIGH);
      analogWrite(en2, 50);
      while (1) {
        read_IR();
        if (right_IR == HIGH) {

          Serial.println("left detect");
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
void go_reverse() {
  steps = 0;
  while (1) {

    line_follow(reverse);
    Serial.println("go straight");
    if (steps >= 300) {
      Serial.println("Start check 2nd cross");
      break;
    }

  }

}
void check_launchzone() {
  while (1) {
    line_follow(forward);
    read_IR();
    Serial.println(left_IR);
    Serial.println(right_IR);

    if (left_IR == HIGH ) {
      Serial.println("left detect");
      analogWrite(en1, 0);
      digitalWrite(dir2, LOW);
      analogWrite(en2, 50);
      while (1) {
        read_IR();
        if (right_IR == HIGH) {
          Serial.println("right detect");
          analogWrite(en2, 0);
          launch_flag = 1;
          delay(5000);
          break;
        }

      }
      break;
    }
    if (right_IR == HIGH) {
      Serial.println("right detect");
      analogWrite(en2, 0);
      digitalWrite(dir1, LOW);
      analogWrite(en1, 50);
      while (1) {
        read_IR();
        if (left_IR == HIGH) {

          Serial.println("left detect");
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
void go_straight() {
  steps = 0;
  while (1) {

    //line_follow(forward);
    Serial.println("go straight");
    if (steps >= 300) {
      Serial.println("Start check 2nd cross");
      break;
    }

  }

}
void line_follow(int dir) {
  //dir2 is right, 1 is backward

  while (1) {
    WA_direction = digitalRead(leftright);
    multiply1 = digitalRead(power1);
    multiply2 = digitalRead(power2);
    multiply3 = digitalRead(power3);
    junction_flag = digitalRead(junction);
    if (junction_flag) {
      motorStop();
      go_straight();
      break;
    }
    if (dir == reverse) {
      if (close_flag == 0) {
        check_ultrasonic1();
      } else {
        check_ultrasonic2();
      }
    }
    if (WA_direction == HIGH) { //go right
      if (multiply1 == HIGH) {


        rightspeed = 50;
        leftspeed = normal_speed;
      }
      if (multiply2 == HIGH) {

        rightspeed = 60;
        leftspeed = normal_speed;
      }
      if (multiply3 == HIGH) {

        rightspeed = 70;
        leftspeed = normal_speed;
      }
      if (multiply1 == LOW && multiply2 == LOW && multiply3 == LOW) {
        rightspeed = normal_speed;
        leftspeed = normal_speed;
      }
    }

    if (WA_direction == LOW) { //go left
      if (multiply1 == HIGH) {

        leftspeed = 50;
        rightspeed = normal_speed;
      }
      if (multiply2 == HIGH) {


        leftspeed = 60;
        rightspeed = normal_speed;
      }
      if (multiply3 == HIGH) {
        //digitalWrite(led15, LOW);

        leftspeed = 70;
        rightspeed = normal_speed;
      }
      if (multiply1 == LOW && multiply2 == LOW && multiply3 == LOW) {
        rightspeed = normal_speed;
        leftspeed = normal_speed;
      }
    }
    digitalWrite(dir1, dir);
    digitalWrite(dir2, dir);
    analogWrite(en2, rightspeed);
    analogWrite(en1, leftspeed);
  }
}



void ignorecross() {
  while (1) {
    Serial.println("In ignore");
    line_follow(forward);

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

void turnLeftcross() {
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  digitalWrite(dir2, LOW);
  analogWrite(en2, 75);
  digitalWrite(dir1, HIGH);
  analogWrite(en1, 75);
  checkcross();
}

void turnRightcross() {
  attachInterrupt(digitalPinToInterrupt(encoder1), cal, CHANGE);
  digitalWrite(dir2, HIGH);
  analogWrite(en2, 75);
  digitalWrite(dir1, LOW);
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
      steps = 0;
      break;
    }
  }
}

void smallreverse() {
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




void cal() {
  steps++;
  Serial.println(steps);
  if (steps >= 10000) {
    steps = 0;
  }

}

void turnLeft90() {
  while (1) {
    line_follow(forward);
    Serial.println(sumvalue);
    if (sumvalue >= cross_value ) {
      smallreverse();
      turnLeftcross();
      break;
    }
  }
}

void turnRight90() {
  while (1) {
    line_follow(forward);
    //Serial.print(rightMost);
    //Serial.print("  ");
    //Serial.print(right2Most);
    //Serial.print("  ");
    //Serial.println(right3Most);

    if (rightMost >= 70 && right2Most >= 70 && right3Most >= 70) {
      smallreverse();
      turnRightcross();
      break;
    }
  }
}


