#include "Arduino.h"
#include "Serial_Arduino_to_Arduino.h"

void Com::Master()
{
  Serial.write((int)1);
  delay(100);
}

