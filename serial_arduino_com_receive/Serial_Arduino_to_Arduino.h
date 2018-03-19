#include "Arduino.h"
#ifndef Serial_Arduino_to_Arduino_h
#define Serial_Arduino_to_Arduino_h

class Com
{
  public:
    void Master();
    void Slave();
    int S;
    int R;
};

#endif
