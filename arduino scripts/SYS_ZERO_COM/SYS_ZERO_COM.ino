#include<SoftwareSerial.h>
//#include"FABRIK.h"
#include<math.h>
#include"FABRIK_Solver.h"
#define Pi 3.1415926535

const byte txPin = 2;
const byte rxPin = 3;
SoftwareSerial mySerial1(rxPin, txPin);
FABRIK_Solver mySolver = FABRIK_Solver();

//Vector3d inputPositionJoints[7];
Vector3d targetPosition1;
Vector3d targetOrientation[3];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  while (Serial.available() > 0)
  {
    Serial.read();
  }

  mySerial1.begin(9600);
  mySolver.Solve();
}



void loop() {
  if (Serial.available() > 0)
  {
    Serial.println("Start reading");
    String comHolder = "";
    int comNumber = 0;
    char charsTemp[100];
    
    //while (Serial.available() > 0)
    //{
    //  comHolder += char(Serial.read());
    //  delay(2);
    //}
    delay(20);
    comNumber = Serial.readBytesUntil('e', charsTemp, 100);
    Serial.println(comNumber);
    for(int i=0; i<comNumber;i++)
    {
      comHolder += charsTemp[i];
    }
    //Serial.println(comHolder);

    Serial.println("End reading");
    delay(2);

    double cmds[12];//com like this is ok
    for (int i = 0; i < 12; i++)//I shall head for the next step
      cmds[i] = 0;
    int indexcmds = 0;
    int pointPos = 0;
    int spacePos = 0;
    bool countflag = true;
    bool negflag = false;

    for (int i = 0; i < comHolder.length(); i++)
    {
      if (comHolder[i] == ' ')
      {
        spacePos = i;
        if (negflag == true)
        {
          cmds[indexcmds] = -cmds[indexcmds];
        }
        if (countflag == false)
        {
          cmds[indexcmds] = cmds[indexcmds] / pow(10, spacePos - pointPos - 1);;
        }
        indexcmds++;
        countflag = true;
        negflag = false;
        continue;
      }
      if (comHolder[i] == '-')
      {
        negflag = true;
        continue;
      }
      if (comHolder[i] == '.')
      {
        countflag = false;
        pointPos = i;
        continue;
      }
      cmds[indexcmds] = cmds[indexcmds] * 10 + (comHolder[i] - '0');
    }

    targetPosition1 = {cmds[0], cmds[1], cmds[2]};//this works
    targetOrientation[0] = { cmds[3], cmds[4], cmds[5] };
    targetOrientation[1] = { cmds[6], cmds[7], cmds[8] };
    targetOrientation[2] = { cmds[9], cmds[10], cmds[11] };
    mySolver.UpdateTarget(targetPosition1, targetOrientation);
    mySolver.Solve();

    int sign[7] = { -1, -1, 1, -1, 1, -1, 1};
    int outNum[7];

    Serial.println(comHolder.length());

    for (int i = 0; i < 12; i++)
    {
      Serial.println(cmds[i]);
    }

    for (int i = 0; i < 7; i++)
    {
      Serial.println(mySolver.GetAngle(i, sign[i]));
      outNum[i] = 1500 + (mySolver.GetAngle(i, sign[i]) / (Pi / 2)) * 1000;
      //Serial.println(outNum[i]);
    }

    /*
      unsigned char Ch0_Pos1500[10] = {0xff, 0x01, 0x00, 0x14, 0x00,0xff, 0x02, 0x00, outNum[0] & 0xff, (outNum[0] & 0xff00) >> 8 };
      unsigned char Ch1_Pos1500[10] = {0xff, 0x01, 0x01, 0x14, 0x00,0xff, 0x02, 0x01, outNum[1] & 0xff, (outNum[1] & 0xff00) >> 8 };
      unsigned char Ch2_Pos1500[10] = {0xff, 0x01, 0x02, 0x14, 0x00,0xff, 0x02, 0x02, outNum[2] & 0xff, (outNum[2] & 0xff00) >> 8 };
      unsigned char Ch3_Pos1500[10] = {0xff, 0x01, 0x03, 0x14, 0x00,0xff, 0x02, 0x03, outNum[3] & 0xff, (outNum[3] & 0xff00) >> 8 };
      unsigned char Ch4_Pos1500[10] = {0xff, 0x01, 0x04, 0x14, 0x00,0xff, 0x02, 0x04, outNum[4] & 0xff, (outNum[4] & 0xff00) >> 8 };
      unsigned char Ch5_Pos1500[10] = {0xff, 0x01, 0x05, 0x14, 0x00,0xff, 0x02, 0x05, outNum[5] & 0xff, (outNum[5] & 0xff00) >> 8 };
      unsigned char Ch6_Pos1500[10] = {0xff, 0x01, 0x06, 0x14, 0x00,0xff, 0x02, 0x06, outNum[6] & 0xff, (outNum[6] & 0xff00) >> 8 };
    */

    unsigned char Chs_MidPos1500[35];
    for (unsigned char i = 0; i <= 6; i++)
    {
      Chs_MidPos1500[i * 5 + 0] = 0xff;
      Chs_MidPos1500[i * 5 + 1] = 0x02;
      Chs_MidPos1500[i * 5 + 2] = i;
      Chs_MidPos1500[i * 5 + 3] = outNum[i] & 0xff;
      Chs_MidPos1500[i * 5 + 4] = (outNum[i] & 0xff00) >> 8;
    }


    //mySerial1.write(Ch0_Pos1500+Ch1_Pos1500[5]+Ch2_Pos1500[5]+Ch3_Pos1500[5]+Ch4_Pos1500[5]+Ch5_Pos1500[5]+Ch6_Pos1500[5], 70);
    //delay(200);

    mySerial1.write(Chs_MidPos1500, 35);

    Serial.println('g');

    //Serial.println(comNumber);
    //if(comNumber<1000)
    //  comNumber=1000;
    //if(comNumber>2000)
    //  comNumber=2000;
    //Serial.println(comNumber);

    //Ch0_Pos1500[4]=(comNumber & 0xff00)>>8;
    //Serial.println(Ch0_Pos1500[4]);
    //Ch0_Pos1500[3]=comNumber & 0xff;
    //Serial.println(Ch0_Pos1500[3]);
    //mySerial1.write(Ch0_Pos1500, 5);
    // put your main code here, to run repeatedly:
  }
}
