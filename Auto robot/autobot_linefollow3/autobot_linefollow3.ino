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
int normal_speed = 30;
volatile int steps = 0;     //Encoder starting values
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;

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

// camera

int led16 = 51;
int led17 = 53;

//Flags
int ignore_flag = 0;
int launch_flag = 0;
int reload_flag = 0;
int close_flag = 0;
int stop_flag = 0;

//Unused
int magic_number  = 40;

void setup() {
  Serial.begin(115200);
  //PinMode setup
  pinMode(encoder1, INPUT_PULLUP);
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
  pinMode(led16, OUTPUT);
  pinMode(led17, OUTPUT);     // Yellow LED

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

void line_follow(int dir) {
  //dir2 is right, 1 is backward
  Serial.println("kk");
  digitalWrite(led16,HIGH);
  digitalWrite(led17,HIGH);
  while (1) {
    WA_direction = digitalRead(leftright);
    multiply1 = digitalRead(power1);
    multiply2 = digitalRead(power2);
    multiply3 = digitalRead(power3);

    if (WA_direction == HIGH) { //go right
      if (multiply1 == HIGH) {
        digitalWrite(led10, LOW);
        digitalWrite(led1, HIGH);
        rightspeed = 50;
        leftspeed = normal_speed;
      }
      if (multiply2 == HIGH) {
        digitalWrite(led1, LOW);
        digitalWrite(led5, HIGH);
        rightspeed = 60;
        leftspeed = normal_speed;
      }
      if (multiply3 == HIGH) {
        digitalWrite(led5, LOW);
        digitalWrite(led8, HIGH);
        rightspeed = 70;
        leftspeed = normal_speed;
      }
    }

    if (WA_direction == LOW) { //go left
      if (multiply1 == HIGH) {
        digitalWrite(led8, LOW);
        digitalWrite(led13, HIGH);
        leftspeed = 50;
        rightspeed = normal_speed;
      }
      if (multiply2 == HIGH) {
        digitalWrite(led13, LOW);
        digitalWrite(led15, HIGH);
        leftspeed = 60;
        rightspeed = normal_speed;
      }
      if (multiply3 == HIGH) {
        digitalWrite(led15, LOW);
        digitalWrite(led10, HIGH);
        leftspeed = 70;
        rightspeed = normal_speed;
      }
    }
    digitalWrite(dir1, dir);
    digitalWrite(dir2, dir);
    analogWrite(en2, rightspeed);
    analogWrite(en1, leftspeed);
  }
}






