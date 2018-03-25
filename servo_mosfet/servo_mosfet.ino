#include <Servo.h>
int mosfet = 8;

Servo myservo;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(mosfet,OUTPUT);
  myservo.attach(9);
  myservo.write(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(mosfet,HIGH);
  Serial.println("High");
  delay(5000);
  digitalWrite(mosfet,LOW);
  Serial.println("Low");
  delay(5000);
}
