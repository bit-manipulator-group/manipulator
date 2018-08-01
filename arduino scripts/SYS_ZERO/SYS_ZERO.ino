#include<SoftwareSerial.h>
#include<math.h>

const byte txPin = 2;
const byte rxPin = 3;
SoftwareSerial mySerial1(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (Serial.available() > 0)
  {
    Serial.read();
  }
  mySerial1.begin(9600);
}
byte se_Part1;


void loop() {
  if (Serial.available() > 0)
  {
    String comHolder = "";
    int comNumber = 0;

    while (Serial.available() > 0)
    {
      comHolder += char(Serial.read());
      delay(2);
    }

    for (int i = 0; i < comHolder.length(); i++)
    {
      comNumber = comNumber * 10 + (comHolder[i] - '0');
    }
    Serial.println(comNumber);
    if (comNumber < 500)
      comNumber = 500;
    if (comNumber > 2500)
      comNumber = 2500;
    Serial.println(comNumber);
    unsigned char Ch0_Ch1_Pos1500[35];
    //{0xff, 0x02, 0x01, 0xdc, 0x05, 0xff, 0x02, 0x00, 0xdc, 0x05 }
    for(unsigned char i=0;i<=6;i++)
    {
      Ch0_Ch1_Pos1500[i*5+0] = 0xff;
      Ch0_Ch1_Pos1500[i*5+1] = 0x02;
      Ch0_Ch1_Pos1500[i*5+2] = i;
      Ch0_Ch1_Pos1500[i*5+3] = comNumber & 0xff;
      Ch0_Ch1_Pos1500[i*5+4] = (comNumber & 0xff00) >> 8;
    }

    //Ch0_Ch1_Pos1500[4] = (comNumber & 0xff00) >> 8;
    //Ch0_Ch1_Pos1500[3] = comNumber & 0xff;
    //Ch0_Ch1_Pos1500[9] = (comNumber & 0xff00) >> 8;
    //Ch0_Ch1_Pos1500[8] = comNumber & 0xff;
    //Serial.println(Ch0_Ch1_Pos1500);
    mySerial1.write(Ch0_Ch1_Pos1500, 15);
    // put your main code here, to run repeatedly:
  }
}
