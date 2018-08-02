void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(2);

  while (Serial.available() > 0)
  {
    Serial.read();
  }

  delay(2);

  Serial.println("Serial Port Initiated.");



}
String strGets = "";

void loop() {
  if (Serial.available() > 0)
  {
    if (Serial.peek() != 'e')
    {
      strGets += char(Serial.read());
    }
    else
    {
      while (Serial.available() > 0)
      {
        Serial.read();
        delay(2);
      }
      //execute main process

      //delay(50);
      //delay(50);
      //delay(900);
      //execute main process
      Serial.println(strGets);
      Serial.println("g");
      strGets = "";
    }
  }
}
