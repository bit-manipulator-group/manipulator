#include <Servo.h>
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int minPluseTime = 1000;
int maxPluseTime = 2000;
int NumSteps = 6;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos = 0; pos < NumSteps; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.writeMicroseconds(minPluseTime+(maxPluseTime-minPluseTime)/(NumSteps-1)*pos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15ms for the servo to reach the position
  }
}
