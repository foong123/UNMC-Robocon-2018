#include "Arduino.h"
#include "Serial_Arduino_to_Arduino.h"

void Com::Slave()
{
  int Rec = Serial.read();
  Serial.println(Rec);
  delay(100);
}
