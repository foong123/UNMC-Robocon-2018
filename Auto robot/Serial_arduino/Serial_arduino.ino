int ledPin12 = 12;
int ledPin6 = 6;
int ledPin4 = 4;
int ledPin2 = 2;
void light(int n);

void setup()
{
  pinMode(ledPin12, OUTPUT);
  pinMode(ledPin6, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  while (Serial.available()>0) 
  {
  int p;
  p = Serial.readStringUntil('s');
  Serial.println(p);
  light(Serial.readStringUntil('s')); 
  // convert any character that represents a digit to the number it represents. 
  //Serial.flush();
  }
}

void light(int n)
{
  Serial.println(n);
  if (n == 1)
  {
  digitalWrite(ledPin12, HIGH);
  delay(500);
  }
  if (n == 2)
  {
  digitalWrite(ledPin6, HIGH);
  delay(500);
  }
  if (n == 3)
  {
  digitalWrite(ledPin4, HIGH);
  delay(500);
  }
  if (n == 4)
  {
  digitalWrite(ledPin2, HIGH);
  delay(500);
  digitalWrite(ledPin2, LOW);
  delay(500);
  }
}
