#include<SoftwareSerial.h>
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
String strGets = "";
int numGets = 0;

int indexcmds = 0;
int pointPos = 0;
int spacePos = 0;
bool countflag = true;
bool negflag = false;
double cmds[12];

int sign[7] = { -1, -1, 1, -1, 1, -1, 1};
int outNum[7];

unsigned char Chs_MidPos1500[35];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(2);

  while (Serial.available() > 0)
  {
    Serial.read();
  }

  delay(2);

  mySerial1.begin(9600);
  mySolver.Solve();
  Serial.println("Serial Port Initiated.");
}

void loop() {

  //execute main process

  for (int i = 0; i < 12; i++)//this works
    cmds[i] = 0;

  spacePos = 0;
  pointPos = 0;
  countflag = true;
  negflag = false;
  indexcmds = 0;
  for (int i = 0; i < numGets; i++)
  {
    if (strGets[i] == ' ')
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
    if (strGets[i] == '-')
    {
      negflag = true;
      continue;
    }
    if (strGets[i] == '.')
    {
      countflag = false;
      pointPos = i;
      continue;
    }
    cmds[indexcmds] = cmds[indexcmds] * 10 + (strGets[i] - '0');
  }

  targetPosition1 = {cmds[0], cmds[1], cmds[2]};//this works
  targetOrientation[0] = { cmds[3],   cmds[4],  cmds[5] };
  targetOrientation[1] = { cmds[6],   cmds[7],  cmds[8] };
  targetOrientation[2] = { cmds[9],   cmds[10], cmds[11] };
  mySolver.UpdateTarget(targetPosition1, targetOrientation);
  mySolver.Solve();

  for (int i = 0; i < 7; i++)
  {
    Serial.println(mySolver.GetAngle(i, sign[i]));
    //delay(2);
    outNum[i] = 1500 + (mySolver.GetAngle(i, sign[i]) / (Pi / 2)) * 1000;
    //Serial.println(outNum[i]);
  }

  for (unsigned char i = 0; i <= 6; i++)
  {
    Chs_MidPos1500[i * 5 + 0] = 0xff;
    Chs_MidPos1500[i * 5 + 1] = 0x02;
    Chs_MidPos1500[i * 5 + 2] = i;
    Chs_MidPos1500[i * 5 + 3] = outNum[i] & 0xff;
    Chs_MidPos1500[i * 5 + 4] = (outNum[i] & 0xff00) >> 8;
  }

  mySerial1.write(Chs_MidPos1500, 35);

  //execute main process
  //Serial.println(strGets);
  Serial.println("g");
  strGets = "";
  numGets = 0;
}

void serialEvent()
{
  while (Serial.peek() != 'e' && Serial.peek() != -1)
  {
    strGets += char(Serial.read());
    numGets++;
  }
}
