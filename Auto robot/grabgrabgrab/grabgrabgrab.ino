#include <Servo.h>
#define start_to_grab 40
#define start_to_grab_TZ1 37
#define start_to_grab_TZ2 38
#define start_to_grab_TZ3 39
#define can_move_ady 36
#define trigPin1 13
#define echoPin1 12
#define trigPin2 4
#define echoPin2 2

int counter = 0;
int input = A8;
int k = 26 * 200;
int i = 0;

long duration1, duration2, distance1, distance2;
volatile int steps = 0;
double angle_per_steps = 360 / 400;
int motorA = 5;
int motorB = 6;
double release_angle = 135;
double error = 1.0;
Servo myservo;

void setup() {
  Serial.begin(115200);
  myservo.attach(9);
  pinMode(start_to_grab, INPUT);
  pinMode(launch_TZ1, INPUT);
  pinMode(launch_TZ2, INPUT);
  pinMode(launch_TZ3, INPUT);
  pinMode(can_move_ady, OUTPUT);
  pinMode(input, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  myservo.write(100);
  //if (Serial.read() == 'c') {
  if (digitalRead(start_to_grab) == 1) {
    // reloading
    grab_grab();
  }
  else if (digitalRead(launch_TZ1) == 1)
  {
    //TZ1 launching
    TZ1();
  }
  else if (digitalRead(launch_TZ2) == 1)
  {
    //TZ2 launching
    TZ2();
  }
  else if (digitalRead(launch_TZ3) == 1)
  {
    //TZ3 launching
    TZ3();
  }
}

void grab_grab() {
  motorstart(14, 0);
  Serial.println("motor clamping start");
  while (1) {
    check_ultrasonic1();
    double data = k / analogRead(input);
    Serial.print(data);
    Serial.print("  ");
    Serial.println(distance2);
    if (data <= 10 ) {
      i++;
      if (i >= 0 || distance1 == 11) {
        //delay(50);
        myservo.write(40);
        delay(1300);
        Serial.println("servo Closed");
        //motorstop();
        motorstart(8, 0);
        delay(3000);
        motorstart(35, 0);
        delay(700);
        motorstart(0, 3);
        delay(200);
        while (1) {
          motorstart(10, 0);
          check_ultrasonic2();
          if (distance2 <= 18 && distance2 >= 13) {
            motorstop();
            delay(4000);
          }
        }
      }
    }
  }
}

void TZ1() {
  motorstart(0 , 180);
  Serial.println("motor launching start");
  delay(182);
  myservo.write(100);
  motorstop();
  digitalWrite(can_move_ady, HIGH);
  while (1);
  i = 0;
}

void TZ2() {
  motorstart(0 , 190);
  Serial.println("motor launching start");
  delay(205);
  myservo.write(100);
  motorstop();
  digitalWrite(can_move_ady, HIGH);
  while (1);
  i = 0;
}

void TZ3() {
  motorstart(0 , 240);
  Serial.println("motor launching start");
  delay(165);
  myservo.write(100);
  motorstop();
  digitalWrite(can_move_ady, HIGH);
  while (1);
  i = 0;
}

void motorstart(int motorspeeda, int motorspeedb) {
  analogWrite(motorA , motorspeeda);
  analogWrite(motorB , motorspeedb);
}

void motorstop() {
  analogWrite(motorA , 0);
  analogWrite(motorB , 0);
  Serial.println("Motor stopped");
}

void cal() {
  steps ++;
  /* faster version
    if(steps >= release_steps - step_error && steps <= release_steps + step_error ){
    // release servo
    }
  */
  if (steps * angle_per_steps >= release_angle - error && steps * angle_per_steps <= release_angle + error) {
    //servo release
    // myservo.write(100);
    //motorstop();
  }

}


void check_ultrasonic1() {
  delay(50);
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1 / 2) / 29.1;
}

void check_ultrasonic2() {
  delay(50);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2 / 2) / 29.1;
}
