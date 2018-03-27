int ADS1 = A0;
int ADS2 = A1;
int ADS3 = A2;
double k1 = 200*26;
double k3 = 200*26;
void setup() {
Serial.begin(9600);
pinMode (ADS1,INPUT);
pinMode (ADS2,INPUT);
pinMode (ADS3,INPUT);

}

void loop() {
  int input1 = k1/analogRead(ADS1);
  int input2 = k1/analogRead(ADS2);
  int input3 = k1/analogRead(ADS3);
  Serial.print(input1);
  Serial.print("  ");
   Serial.print(input2);
  Serial.print("  ");
   Serial.print(input3);
  Serial.println("  ");
  delay(500);
}
