#include<SoftwareSerial.h>

const byte txPin = 2;
const byte rxPin = 3;
SoftwareSerial mySerial1(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(Serial.available()>0)
  {
    Serial.read(); 
  }
  mySerial1.begin(9600);
}
byte se_Part1;
unsigned char Ch0_Pos1500[5] = {0xff, 0x02, 0x00, 0xdc, 0x05 };

void loop() {
  if(Serial.available()>0)
  {
  String comHolder = "";
  int comNumber=0;

  while(Serial.available()>0)
  {
    comHolder += char(Serial.read());
    delay(2);
  }
  for(int i=0;i<comHolder.length();i++)
  {
     comNumber = comNumber*10+(comHolder[i]-'0');
  }
  Serial.println(comNumber);
  if(comNumber<1000)
    comNumber=1000;
  if(comNumber>2000)
    comNumber=2000;
  Serial.println(comNumber);

  Ch0_Pos1500[4]=(comNumber & 0xff00)>>8;
  Serial.println(Ch0_Pos1500[4]);
  Ch0_Pos1500[3]=comNumber & 0xff;
  Serial.println(Ch0_Pos1500[3]);
  mySerial1.write(Ch0_Pos1500, 5);
  // put your main code here, to run repeatedly:
  }
}
