#include <Servo.h>
#include <NewPing.h>

int trig1 = 13; //blue jumper wire (front)
int echo1 = 12; //green jumper wire(front)
int trig2 = 4; //yellow jumper wire(back)
int echo2 = 3; //red jumper wire(back)
int counter = 0;
int input = A8;
int k = 26 * 200;
int i = 0;
double sonar1 = 0;
double sonar2 = 0;
int close_flag = 0;
int max_dist = 30;
double manual_dist = 20.0;
NewPing ultrasonic1(trig1, echo1, max_dist); // NewPing setup of pins and maximum distance.
NewPing ultrasonic2(trig2, echo2, max_dist); // NewPing setup of pins and maximum distance.

int TZ1_pin = 38;
int TZ2_pin = 41;
int TZ3_pin = 40;
int reload_pin = 37;
int receive_pin = 36;

int encoderA = 2;
int encoderB = 3;
volatile int steps = 0;
double angle_per_steps = 360 / 400;
int motorA = 5;
int motorB = 6;
double release_angle = 135;
double error = 1.0;
Servo myservo;

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  pinMode(input, INPUT);
  pinMode(TZ1_pin, INPUT);
  pinMode(TZ2_pin, INPUT);
  pinMode(TZ3_pin, INPUT);
  pinMode(receive_pin, OUTPUT);
  pinMode(reload_pin, INPUT);
}

void loop() {
  int TZ1 = 0;
  int TZ2 = 0;
  int TZ3 = 0;
  int reload = 0;

  reload = digitalRead(reload_pin);
  if (reload == HIGH) {
    myservo.write(100);
    motorstart(14, 0);
    Serial.println("motor clamping start");
    while (1) {
      check_ultrasonic1();
      double data = k / analogRead(input);
      Serial.print(data);
      Serial.print("  ");
      Serial.println(sonar1);
      if (data <= 9 ) {
        i++;
        if (i >= 0 || sonar1 == 11) {
          //delay(50);
          myservo.write(40);
          delay(1300);
          Serial.println("servo Closed");
          motorstart(8, 0);
          digitalWrite(receive_pin, HIGH);
          delay(3000);
          digitalWrite(receive_pin, LOW);
          motorstart(35, 0);
          delay(500);
          while (1) {
            motorstart(8, 0);
            check_ultrasonic2();
            if (sonar2 <= 18 && sonar2 >= 13) {
              motorstop();
              delay(4000);
              while (1) {
                reload = digitalRead(reload_pin);
                TZ1 = digitalRead(TZ1_pin);
                TZ2 = digitalRead(TZ2_pin);
                TZ3 = digitalRead(TZ3_pin);
                if (reload == HIGH && TZ1 == HIGH) {
                  launchTZ1();
                  digitalWrite(receive_pin, HIGH);
                  break;
                }
                if (reload == HIGH && TZ2 == HIGH) {
                  launchTZ2();
                  digitalWrite(receive_pin, HIGH);
                  break;
                }
                if (reload == HIGH && TZ2 == HIGH) {
                  launchTZ3();
                  digitalWrite(receive_pin, HIGH);
                  break;
                }
                digitalWrite(receive_pin, LOW);
                break;
              }
              break;
            }
          }
        }
      }
      else {
        i = 0;
      }
    }
  }
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
  sonar1 = ultrasonic1.ping_cm();
  //Serial.print("1st ultrasonic : ");
  //Serial.println(sonar1);
  /*if (sonar1 <= manual_dist && sonar1 > 0) {
    Serial.println("lst detected");
    close_flag = 1;
    }*/
}

void check_ultrasonic2() {
  delay(50);
  sonar2 = ultrasonic2.ping_cm();
  Serial.print("2st ultrasonic : ");
  Serial.println(sonar2);
  /*if (sonar1 <= manual_dist && sonar1 > 0) {
    Serial.println("lst detected");
    close_flag = 1;
    }*/
}

void launchTZ1() {
  motorstart(0 , 130);
  Serial.println("motor launching start");
  delay(260);
  myservo.write(100);
  motorstop();
}

void launchTZ2() { //to be test
  motorstart(0 , 130);
  Serial.println("motor launching start");
  delay(260);
  myservo.write(100);
  motorstop();
}

void launchTZ3() { //to be test
  motorstart(0 , 130);
  Serial.println("motor launching start");
  delay(260);
  myservo.write(100);
  motorstop();
}

