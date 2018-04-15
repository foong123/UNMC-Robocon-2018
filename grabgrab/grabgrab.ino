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
  //pinMode(encoderA , INPUT_PULLUP);
  //pinMode(encoderB , INPUT_PULLUP);
  // pinMode(motorA , OUTPUT);
  // pinMode(motorB , OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(encoderA),cal,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderB),cal,CHANGE);
  myservo.attach(9);
  pinMode(input, INPUT);
  //  myservo.write(5);
  //Serial.println("Openeed");

  //myservo.write(15);
  //Serial.println("Clamped");

}

void loop() {
  //delay(50);
  Serial.println(ultrasonic1.ping_cm());
  //  if(Serial.read() == 'c'){
  //    motorstart(0 ,200);
  //          Serial.println("motor launching start");
  //          delay(230);
  //          myservo.write(80);
  //          motorstop();
  //
  //          while(1);
  //  }
  //
  //    if(Serial.read() == 'd'){
  //      myservo.write(5);
  //      Serial.println("servo clamped");
  //    }
  //    if(Serial.read() == 'j'){
  //      myservo.write(85);
  //      Serial.println("servo released");
  //    }



  //  // put your main code here, to run repeatedly:
  //  check_ultrasonic1();
  myservo.write(100);
  if (Serial.read() == 'c') {
    motorstart(14, 0);
    Serial.println("motor clamping start");
    while (1) {
      /*
        check_ultrasonic1();
        //if (sonar1 ==9) {
        //counter++;
        //}
        //if (sonar1 ==10 && counter >= 2) {
        if (sonar1 ==11) {
        delay(50);
        myservo.write(40);
        delay(1300);
        Serial.println("servo Closed");
        motorstop();
        delay(6000);
        motorstart(35, 0);
        delay(1000);
        while (1) {
          motorstart(8,0);
          check_ultrasonic2();
          if (sonar2 <= 18 && sonar2 >=13) {
            motorstop();
            delay(4000);
            motorstart(0 , 180);
            Serial.println("motor launching start");
            delay(230);
            myservo.write(100);
            motorstop();

            while (1);
          }
        }
        }*/
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
//          motorstop();
          motorstart(8,0);
          delay(3000);
          motorstart(35, 0);
          delay(500);
          while (1) {
            motorstart(8, 0);
            check_ultrasonic2();
            if (sonar2 <= 18 && sonar2 >= 13) {
              motorstop();
              delay(4000);
              motorstart(0 , 130);
              Serial.println("motor launching start");
              delay(260);
              myservo.write(100);
              motorstop();

              while (1);
            }
          }
        }
      }
      else {
        i = 0;
      }
    }
    //myservo.write(60);

  }
  //
  //



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
