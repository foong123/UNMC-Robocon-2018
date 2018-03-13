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
//-----( Import needed libraries )-----
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN 6
#define CSN_PIN 8

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

#define light_Front  18     //LED Front Right   pin A4 for Arduino Uno
#define light_Back   19     //LED Back Right    pin A5 for Arduino Uno
#define horn_Buzz    7     

#define ENA_m1 3          // Enable/speed motor Front Right 
#define ENB_m1 5          // Enable/speed motor Back Right
#define ENA_m2 9          // Enable/speed motor Front Left
#define ENB_m2 10         // Enable/speed motor Back Left

#define IN_11  0          // L298N #1 in 1 motor Front Right
#define IN_12  1          // L298N #1 in 2 motor Front Right
#define IN_13  2          // L298N #1 in 3 motor Back Right
#define IN_14  4          // L298N #1 in 4 motor Back Right

#define IN_21  14         // L298N #2 in 1 motor Front Left
#define IN_22  15         // L298N #2 in 2 motor Front Left
#define IN_23  16         // L298N #2 in 3 motor Back Left
#define IN_24  17         // L298N #2 in 4 motor Back Left

/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int joystick[6];  // 6 element array holding Joystick readings
int speedRight = 0;
int speedLeft = 0;
int  xAxis, yAxis;
// the four button variables from joystick
int buttonUp;
int buttonRight;
int buttonDown;
int buttonLeft;

void setup()
{
  pinMode(light_Front, OUTPUT);
  pinMode(light_Back, OUTPUT);
  pinMode(horn_Buzz, OUTPUT);
    
  pinMode(ENA_m1, OUTPUT);
  pinMode(ENB_m1, OUTPUT);
  pinMode(ENA_m2, OUTPUT);
  pinMode(ENB_m2, OUTPUT);
  
  pinMode(IN_11, OUTPUT);
  pinMode(IN_12, OUTPUT);
  pinMode(IN_13, OUTPUT);
  pinMode(IN_14, OUTPUT);
    
  pinMode(IN_21, OUTPUT);
  pinMode(IN_22, OUTPUT);
  pinMode(IN_23, OUTPUT);
  pinMode(IN_24, OUTPUT);

/*  Serial.begin(9600);
  Serial.println("Nrf24L01 Receiver Starting"); */
  
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
}

void loop()
{
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
      
      //Serial.println(); // for debugging
      // Fetch the data payload - Debugging code below
  
      if (buttonUp == HIGH){digitalWrite(horn_Buzz, LOW);}
        else {digitalWrite(horn_Buzz, HIGH);}
      if (buttonRight == HIGH){digitalWrite(light_Front, LOW);}
        else {digitalWrite(light_Front, HIGH);}
      if (buttonLeft == HIGH){digitalWrite(light_Back, LOW);}
        else {digitalWrite(light_Back, HIGH);}

// Y-axis used for forward and backward control
    if (yAxis < 470) {
    // Set Right Motors backward
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    // Set Left Motors backward
    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    speedRight  = map(yAxis, 470, 0, 0, 255);
    speedLeft   = map(yAxis, 470, 0, 0, 255);
  }

else if (yAxis > 550) {
    // Set Right Motors forward
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    // Set Left Motors forward
    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
 
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    speedRight  = map(yAxis, 550, 1023, 0, 255);
    speedLeft   = map(yAxis, 550, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    speedRight  = 0;
    speedLeft   = 0;
  }

// X-axis used for left and right control
  if (xAxis < 470) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(xAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    speedLeft = speedLeft - xMapped;
    speedRight = speedRight + xMapped;
    // Confine the range from 0 to 255
    if (speedLeft < 0) {
      speedLeft = 0;
    }
    if (speedRight > 255) {
      speedRight = 255;
    }
  }
  if (xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    speedLeft = speedLeft + xMapped;
    speedRight = speedRight - xMapped;
    // Confine the range from 0 to 255
    if (speedLeft > 255) {
      speedLeft = 255;
    }
    if (speedRight < 0) {
      speedRight = 0;
    }
  }
  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (speedLeft < 70) {
    speedLeft = 0;
  }
  if (speedRight < 70) {
    speedRight = 0;
  }

  analogWrite(ENA_m1, speedRight); // Send PWM signal to motor A
  analogWrite(ENB_m1, speedRight);
  analogWrite(ENA_m2, speedLeft); // Send PWM signal to motor B
  analogWrite(ENB_m2, speedLeft);
     
  /*    Serial.print("X = ");
      Serial.print(xAxis);
      Serial.print(" Y = ");  
      Serial.print(yAxis);
      Serial.print(" Up = ");
      Serial.print(joystick[2]);
      Serial.print(" Right = ");  
      Serial.print(joystick[3]);
      Serial.print(" Down = ");
      Serial.print(joystick[4]);
      Serial.print(" Left = ");  
      Serial.println(joystick[5]); */
      
      
      
      }

    

  
      
  }













  
