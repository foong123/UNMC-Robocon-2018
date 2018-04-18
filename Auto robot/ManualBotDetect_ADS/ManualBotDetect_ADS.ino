#define start_to_grab 37
#define can_move_ady 36
#define ADS_pin A0

int k = 13 * 200;
long InitialRightSensor, CurrentRightSensor, InitialTime;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(start_to_grab, OUTPUT);
  pinMode(can_move_ady, INPUT);
  pinMode(ADS_pin, INPUT);
}

void loop() {
//  double data = k / analogRead(ADS_pin);
  InitialRightSensor = k / analogRead(ADS_pin);
  for (int i = 0; i < 4; i++)
  {
    CurrentRightSensor = k / analogRead(ADS_pin);
    if (CurrentRightSensor == 1300 || InitialRightSensor == 1300) {
      i = 0;
    } else if (InitialRightSensor == CurrentRightSensor && CurrentRightSensor != 1300) {
      InitialTime = millis();
      while (millis() - InitialTime < 500);
      Serial.println ("Aligned for now");
    } else {
      i = 0;
      Serial.println ("Misaligned");
    }
    Serial.print(InitialRightSensor);
    Serial.print ("         ");
    Serial.println (CurrentRightSensor);
    InitialRightSensor = CurrentRightSensor;
    delay(50);
  }
  Serial.println("Aligned");
  Serial.println("grabgrab");
  digitalWrite(start_to_grab, HIGH);
  

//  while (digitalRead(can_move_ady) == 0);
}


