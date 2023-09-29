#include <EEPROM.h>
#include <Arduino.h>

#ifdef useServo
#include <Servo.h>
Servo servo;
#endif

void Move(int Mode);
#ifdef useDC
void EncoderUpdate();
//void Correct();
void SpeedCalculator();
void ServoEmulator();
#endif

#ifdef Three-Pos
long startpoint;
#endif
long endpoint;
long midpoint;

#ifdef useDC
 unsigned long last;
// double minspeed;
 long lastpos;
 double curspeedvalue;
 double curspeed;
 short startpower;
// double speedgain;
 short Direction;
 //short minspeedValue;
 const float degreespertick=360/PPR;
 volatile double curpoint = 0;
 const short ticktolerance = tolerance / degreespertick;
  short stage;
    bool Attach;
        double targetpoint;
        unsigned long waitStart;
      //  double wantedpoint;
      //  double distance;
#endif

#ifdef useServo
double curpoint;
const float degreespernumber = ServoRange/180;
#endif




