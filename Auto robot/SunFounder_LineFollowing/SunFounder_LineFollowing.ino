#include <Wire.h>
#define uchar unsigned char
uchar t; 
float sumvalueweight;
float sumvalue;
float d;
uchar Weight[9];

//void send_data(short a1,short b1,short c1,short d1,short e1,short f1); 
uchar data[16];

void setup()
{
  
  Wire.begin(); // join i2c bus (address optional for master) 
  Serial.begin(115200); // start serial for output t = 0;
}
void loop()
{
  Wire.requestFrom(9, 16); // request 16 bytes from slave device #9 while (Wire.available()) // slave may send less than requested
  while (Wire.available())
  {
  
    data[t] = Wire.read(); // receive a byte as character 
    if (t < 15)
    t++;
    else
      t = 0;
  }
  Serial.print("data[1]:");
  Serial.println(data[0]);
  Serial.print("data[2]:");
  Serial.println((data[2]*1.02));
  Serial.print("data[3]:");
  Serial.println((data[4]*1.11));
  Serial.print("data[4]:");
  Serial.println((data[6]*0.99));
  Serial.print("data[5]:");
  Serial.println((data[8]*0.81));
  Serial.print("data[6]:");
  Serial.println((data[10]*0.88));
  Serial.print("data[7]:");
  Serial.println((data[12]*0.82));
  Serial.print("data[8]:");
  Serial.println((data[14]*0.89));
 
  sumvalueweight = ((data[0]*(-42))+(data[2]*(-30.6))+(data[4]*(-19.98))+(data[6]*(-5.94))+(data[8]*4.86)+(data[10]*15.84)+(data[12]*24.6)+(data[14]*37.38));
  sumvalue= ((data[0]*1)+(data[2]*1.02)+(data[4]*1.11)+(data[6]*0.99)+(data[8]*0.81)+(data[10]*0.88)+(data[12]*0.82)+(data[14]*0.89));
  d = (sumvalueweight)/(sumvalue);
  Serial.println(d);
  if (sumvalue >= 560.0){
    Serial.println("Turn 90");
  }
   delay(500);
}
