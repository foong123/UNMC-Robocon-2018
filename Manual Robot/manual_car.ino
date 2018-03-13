/* JoyStick module receiver code
 - CONNECTIONS: nRF24L01 Modules See:
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 6
   4 - CSN to Arduino pin 8
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
 */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 6
#define CSN_PIN 8

const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

#define speed_m1 3          // Enable/speed motor Left 
#define speed_m2 5          // Enable/speed motor Right
#define dir_m1 4
#define dir_m2 6

#define magnet1 9
#define magnet2 10

RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

int joystick[6];  // 6 element array holding Joystick readings
int speedRight = 0;
int speedLeft = 0;
int speedx = 0;
int speedy = 0;
int  xAxis, yAxis;
int pwm_value = 155;

int buttonUp;
int buttonRight;
int buttonDown;
int buttonLeft;

bool upstate = false;
bool rightstate = false;
bool downstate = false;
bool leftstate = false;

void setup() {
  pinMode(speed_m1, OUTPUT);
  pinMode(speed_m2, OUTPUT);
  pinMode(dir_m1, OUTPUT);
  pinMode(dir_m2, OUTPUT);

  pinMode(magnet1, OUTPUT);
  pinMode(magnet2, OUTPUT);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
}

void loop() {
  if ( radio.available() )
  {
      radio.read( joystick, sizeof(joystick) );
      xAxis = joystick[0];
      yAxis = joystick[1];
      
      // the four button variables from joystick array
      int buttonUp    = joystick[2];
      int buttonRight = joystick[3];
      int buttonDown  = joystick[4];
      int buttonLeft  = joystick[5];

      if (buttonUp == HIGH){ //turbo mode
        upstate = !upstate; 
        if (upstate = true){
          pwm_value += 100;
        }else{
          pwm_value -= 100;
        }
      }
      if (buttonRight == HIGH){
        rightstate = !rightstate; 
        if (rightstate = true){
          digitalWrite(magnet2, HIGH);
        }else{
          digitalWrite(magnet2, LOW);
        }
      }
      if (buttonDown == HIGH){  //slow down
        downstate = !downstate; 
        if (downstate = true){
          pwm_value -= 100;
        }else{
          pwm_value += 100;
        }
      }
      if (buttonLeft == HIGH){
        leftstate = !leftstate; 
        if (rightstate = true){
          digitalWrite(magnet1, HIGH);
        }else{
          digitalWrite(magnet1, LOW);
        }
      }
      
//backward left and right
  if(yAxis < 470){ 
    speedy   = map(yAxis, 470, 0, 0, 255);
    if(xAxis < 470){ //backward left
      speedx = map(xAxis, 470, 0, 0, 255);
      speedLeft = speedy;                 //here can change to speedy - speedx to increase degree of turning.
      speedRight = speedy + speedx;
    }else if(xAxis > 550){ //backward right
      speedx = map(xAxis, 550, 1023, 0, 255);
      speedLeft = speedy + speedx;
      speedRight = speedy;                //here can change to speedy - speedx to increase degree of turning.
    }else{                //backward
      speedLeft = speedy;
      speedRight = speedy;
    }
    digitalWrite(dir_m1,LOW);
    digitalWrite(dir_m2, LOW);
    analogWrite(speed_m1, speedLeft);
    analogWrite(speed_m2, speedRight);   
  }

//forward left and right
  else if(yAxis >550){ 
    speedy   = map(yAxis, 470, 0, 0, 255);
    if(xAxis < 470){ //forward left
      speedx = map(xAxis, 470, 0, 0, 255);
      speedLeft = speedy;                 //here can change to speedy - speedx to increase degree of turning.
      speedRight = speedy + speedx;
    }else if(xAxis > 550){ //forward right
      speedx = map(xAxis, 550, 1023, 0, 255);
      speedLeft = speedy + speedx;
      speedRight = speedy;                //here can change to speedy - speedx to increase degree of turning.
    }else{                //forward
      speedLeft = speedy;
      speedRight = speedy;
    }
    digitalWrite(dir_m1,HIGH);
    digitalWrite(dir_m2, HIGH);
    analogWrite(speed_m1, speedLeft);
    analogWrite(speed_m2, speedRight);   
  }else{
    digitalWrite(dir_m1,HIGH);
    digitalWrite(dir_m2, HIGH);
    analogWrite(speed_m1, 0);
    analogWrite(speed_m2, 0); 
  }
  
delay(50);




}
