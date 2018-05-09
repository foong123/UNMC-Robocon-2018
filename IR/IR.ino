int A =A1;
int inputVal = 0;
void setup() 
{                
  pinMode(A,INPUT);
  Serial.begin(9600);
}

void loop() 
{
  inputVal = analogRead(A);
  Serial.println(inputVal);
}

