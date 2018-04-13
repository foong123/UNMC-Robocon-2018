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
int junction = 38;

int WA_direction;
int multiply1;
int multiply2;
int multiply3;
int junction_flag = 0;

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
int normal_speed = 50;
volatile int steps = 0;     //Encoder starting values
float leftMotorSpeed = 0;  // Initialise Speed variables
float rightMotorSpeed = 0;

//debugLine

int led1 = 31 ;       // green jumper wire // i2c  // Orange LED

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
int led13 = 49;      // TZ3 go forward + allignment // RED LED
int led14 = 51;      // TZ3 reverse // RED LED

//Ultrasonic
int led16 = 42;      //   WHITE LED
int led17 = 44;      // WHITE LED

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
  Serial.setTimeout(10);
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
  pinMode(led16, OUTPUT);
  pinMode(led17, OUTPUT);     // Yellow LED

  //for line follow
  pinMode(leftright, INPUT);
  pinMode(power1, INPUT);
  pinMode(power2, INPUT);
  pinMode(power3, INPUT);
  pinMode(junction,INPUT);
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
  
  while(Serial.available()){
    int data  = Serial.parseInt();
    if(data >= 0){ //centrod left = -ve
        leftspeed = normal_speed + data/2;
        //rightspeed = normal_speed - data/2;
        rightspeed = 0;
      }else{
        rightspeed = normal_speed + data/2;
        //leftspeed = normal_speed - data/2 ;
        leftspeed = 0;
        }
        digitalWrite(dir1,dir);
        digitalWrite(dir2,dir);
        analogWrite(en1,leftspeed);
        analogWrite(en2,rightspeed);
    }
}






