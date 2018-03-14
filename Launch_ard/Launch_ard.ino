Servo myservo;
int sig;
int motorA1 = 5; //motor pin RPWM
int motorA2 = 6; //motor pin LPWM
int t1 = 0;
int failsave_time = 2000;

void setup() {
Serial.begin(115200);
myservo.attach(9);
pinMode(motorA1,OUTPUT); //set motor pin as output
pinMode(motorA2,OUTPUT);
while(1){
  if(Serial.available()){
    int duty_cycle = Serial.read()-'0';
    directionA(duty_cycle);
    break;
    }
  
  }
  Serial.println("Started");
  t1 = millis();
  
}

void loop() {
  if(Serial.available()){
    sig =  Serial.read() - '0';
    if(sig == 0){
      myservo.write(70);
  
  }else if(sig == 1){
    myservo.write(100);
    Serial.println("Released");
    motorstop();
    Serial.println("Stop");
    while(1);
    }
  }
  if(millis() - t1 >= failsave_time){//fail safe after 2 second
    motorstop();
    }

}
void directionA(int duty_cycle){
  analogWrite(motorA1,duty_cycle); //255 is max speed, can change to other value in 0-255 to control speed.
  analogWrite(motorA2,0);
}
void directionB(int duty_cycle){
  analogWrite(motorA1,0);
  analogWrite(motorA2,duty_cycle);  //255 is max speed, can change to other value in 0-255 to control speed.
}
void motorstop(){
  analogWrite(motorA1,0);
  analogWrite(motorA2,0); 
}
