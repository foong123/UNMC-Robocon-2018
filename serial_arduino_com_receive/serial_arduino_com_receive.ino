#include "Serial_Arduino_to_Arduino.h"

Com com;
void setup() {
  // Begin the Serial at 9600 Baud
  Serial.begin(9600);
}

void loop() {
  com.Slave();
}
