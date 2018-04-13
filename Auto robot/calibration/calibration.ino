#include <Wire.h>
#define uchar unsigned char
uchar t;
float sumvalueweight;
float sumvalue;
float d;
float total[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float average[8];
uchar Weight[9];
float offset[8];
int total_count = 20;



//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
uchar data[16];

void setup()
{

  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(115200); // start serial for output t = 0;
}
void loop()
{
  /*
    Serial.print("data[1]:");
    Serial.println(data[0]);
    Serial.print("data[2]:");
    Serial.println(data[2]);
    Serial.print("data[3]:");
    Serial.println(data[4]);
    Serial.print("data[4]:");
    Serial.println(data[6]);
    Serial.print("data[5]:");
    Serial.println(data[8]);
    Serial.print("data[6]:");
    Serial.println(data[10]);
    Serial.print("data[7]:");
    Serial.println(data[12]);
    Serial.print("data[8]:");
    Serial.println(data[14]);

    //  Weight[1] = -42;
    //  Weight[2] = -30;
    //  Weight[3] = -18;
    //  Weight[4] = -6;
    //  Weight[5] = 6;
    //  Weight[6] = 18;
    //  Weight[7] = 30;
    //  Weight[8] = 42;
    sumvalueweight = ((data[0] * (-42)) + (data[2] * (-30)) + (data[4] * (-18)) + (data[6] * (-6)) + (data[8] * 6) + (data[10] * 18) + (data[12] * 30) + (data[14] * 42));
    sumvalue = (data[0] + data[2] + data[4] + data[6] + data[8] + data[10] + data[12] + data[14]);
    d = (sumvalueweight) / (sumvalue);
    Serial.println(d);
  */

  for (int count = 0; count < total_count; count++) {
    Wire.requestFrom(9, 16); // request 16 bytes from slave device #9 while (Wire.available()) // slave may send less than requested
    while (Wire.available())
    {

      data[t] = Wire.read(); // receive a byte as character
      if (t < 15)
        t++;
      else
        t = 0;
    }

    Serial.print("data[1]:  ");
    Serial.print(data[0]);
    Serial.print("  data[2]:  ");
    Serial.print(data[2]);
    Serial.print("  data[3]:");
    Serial.print(data[4]);
    Serial.print("  data[4]:  ");
    Serial.print(data[6]);
    Serial.print("  data[5]:  ");
    Serial.print(data[8]);
    Serial.print("  data[6]: ");
    Serial.print(data[10]);
    Serial.print("  data[7]:  ");
    Serial.print(data[12]);
    Serial.print("  data[8]:  ");
    Serial.println(data[14]);
    total[0] = total[0] + data[0];
    total[1] = total[1] + data[2];
    total[2] = total[2] + data[4];
    total[3] = total[3] + data[6];
    total[4] = total[4] + data[8];
    total[5] = total[5] + data[10];
    total[6] = total[6] + data[12];
    total[7] = total[7] + data[14];


    delay(500);
  }
  for (int i = 0; i < 8; i++) {
    average[i] = total[i] / total_count;
  }

  /*for (int b = 0; b < 8; b++) {
    Serial.print(average[b]);
    Serial.print("  ");
    }*/
  for (int i = 0; i < 8; i++) {
    offset[i] = average[0] / average[i];
  }

  for (int i = 0; i < 8; i++) {
    Serial.print(offset[i]);
    Serial.print("  ");
  }
  while (true) {
    int a = 1;
  }
}


