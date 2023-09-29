#include "Settings.h"
#include "Main.h"

//EEPROM ARCHITECTURE:
//byte:   usage:
//0-3     start point - 4 bytes
//4-7     mid point - 4 bytes
//8-11    end point - 4 bytes
//12-19   last known point - 8 bytes
//20      is anything set? (255 = no;0=yes) - 1 byte
//21      move start power(DC) - 1 byte
//22      direction (1 = normal, -1 = inversed) (DC) - 1 byte
//23      maxusedspeed value (units) (DC) - 1 byte
//24-31   speed gain per power (tick/(s*Unit)) (DC) - 8 bytes
//32-39   startspeed value (deg/s)  (DC) - 8 bytes



#if  defined (useServo) && defined (useDC)
#error "You have to choose only one of device types (useServo or useDC)"
#endif

#if  !defined (useServo) && !defined (useDC)
#error "You must choose of available device types (useServo or useDC)"
#endif


void setup() {
#ifdef Debug
  Serial.begin(250000);
#endif
#ifdef useServo
  pinMode(servo_pin, OUTPUT);
#endif
#ifdef useDC
  pinMode(ENAdc_pin, OUTPUT);
  pinMode(IN1dc_pin, OUTPUT);
  pinMode(IN2dc_pin, OUTPUT);
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);
#endif
  pinMode(rightbtn_pin, INPUT_PULLUP);
  pinMode(leftbtn_pin, INPUT_PULLUP);
  pinMode(setupbtn_pin, INPUT_PULLUP);
#ifdef useServo
  servo.attach(servo_pin);
#endif
#ifdef useDC
  attachInterrupt(InterruptNumber, EncoderUpdate, CHANGE);
  Attach = false;
#endif
  if (!digitalRead(setupbtn_pin) || EEPROM.read(20) == 255)
  {
#ifdef useServo
#ifdef Three-Pos
    servo.write(90);
    curpoint = 90;
    startpoint = 90;
    while (!digitalRead(setupbtn_pin))
    {
      delay(5);
    }
    while (digitalRead(setupbtn_pin))
    {
      if (!digitalRead(leftbtn_pin) && (curpoint > 0))
      {
        curpoint--;
        startpoint--;
        servo.write(curpoint);
      }
      if (!digitalRead(rightbtn_pin) && (curpoint < 180))
      {
        curpoint++;
        startpoint++;
        servo.write(curpoint);
      }
      delay(30);
    }
#endif
    //---------------------------------------------
    Move(4);
    curpoint = 90;
    midpoint = 90;
    while (!digitalRead(setupbtn_pin))
    {
      delay(5);
    }
    servo.attach(servo_pin);
    servo.write(curpoint);
    while (digitalRead(setupbtn_pin))
    {
      if (!digitalRead(leftbtn_pin) && (curpoint > 0))
      {
        curpoint--;
        midpoint--;
        servo.write(curpoint);
      }
      if (!digitalRead(rightbtn_pin) && (curpoint < 180))
      {
        curpoint++;
        midpoint++;
        servo.write(curpoint);
      }
      delay(30);
    }
    //---------------------------------------------
    Move(4);
    curpoint = 90;
    endpoint = 90;
    while (!digitalRead(setupbtn_pin))
    {
      delay(5);
    }
    servo.attach(servo_pin);
    servo.write(curpoint);
    while (digitalRead(setupbtn_pin))
    {
      if (!digitalRead(leftbtn_pin) && (curpoint > 0))
      {
        curpoint--;
        endpoint--;
        servo.write(curpoint);
      }
      if (!digitalRead(rightbtn_pin) && (curpoint < 180))
      {
        curpoint++;
        endpoint++;
        servo.write(curpoint);
      }
      delay(30);
    }
#endif
#ifdef useDC
    /*
       1. авто запуск 90 градусов (по нажатию на сетап)
       2. Запись напряжения срыва + speed gain
       3. калибровка (едем по напряжению срыва(+5 пунктов))
       4. установка точек
    */
    //<calibration run>
#ifdef Debug
    Serial.println("Started move start measurement...");
#endif
    long lastPos = -5;
    long delta = 0;
    short lastValue = 0;
    while (delta < 3)
    {
      lastValue += 5;
      lastPos = curpoint;
      digitalWrite(IN1dc_pin, HIGH);
      digitalWrite(IN2dc_pin, LOW);
      analogWrite(ENAdc_pin, lastValue);
      delay(500);
      delta = abs(curpoint - lastPos);
    }
    digitalWrite(IN1dc_pin, HIGH);
    digitalWrite(IN2dc_pin, HIGH);
    analogWrite(ENAdc_pin, 255);
    delay(500);
    digitalWrite(IN1dc_pin, LOW);
    digitalWrite(IN2dc_pin, LOW);
    analogWrite(ENAdc_pin, 0);
    startpower = lastValue;
    if (curpoint - lastPos > 0) Direction = 1;
    else Direction = -1;
#ifdef Debug
    Serial.print("Start power = ");
    Serial.println(startpower);
    Serial.print("Direction = ");
    Serial.println(Direction);
#endif
    //--------------------------
    /*   #ifdef Debug
       Serial.println("Started speed gain measurement...");
       #endif
       delta = 0;
       lastValue = startpower;
       long deltaSum = 0;
       int deltaCount = 0;
       long lastdelta = 0;
           digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, LOW);
        analogWrite(ENAdc_pin, lastValue);
        delay(500);
        lastdelta = abs(curpoint-lastPos);
       while (delta*2*degreespertick < maxusedspeed)
       {
        lastValue+= 5;
        lastPos = curpoint;
        digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, LOW);
        analogWrite(ENAdc_pin, lastValue);
        delay(500);
        delta = abs(curpoint-lastPos)*2*degreespertick;
        deltaCount++;
        deltaSum += delta-lastdelta;
        lastdelta = delta;
       }
       speedgain = (deltaSum/deltaCount)/5;
       #ifdef Debug
       Serial.print("Speed gain = ");
       Serial.println(speedgain);
       #endif
       lastValue = startpower;
       analogWrite(ENAdc_pin, lastValue);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
       #ifdef Debug
       Serial.println("Started min speed power value measurement...");
       #endif
       while (delta*2*degreespertick > 1)
       {
        lastValue-= 10;
        lastPos = curpoint;
        digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, LOW);
        analogWrite(ENAdc_pin, lastValue);
        delay(500);
        delta = abs(curpoint-lastPos);
       }
       analogWrite(ENAdc_pin, startpower);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
       lastValue+=10;
          analogWrite(ENAdc_pin, lastValue);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
       while (delta*2*degreespertick > 1)
       {
        lastValue-= 2;
        lastPos = curpoint;
        digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, LOW);
        analogWrite(ENAdc_pin, lastValue);
        delay(500);
        delta = abs(curpoint-lastPos);
       }
          analogWrite(ENAdc_pin, startpower);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
          lastValue+=2;
             analogWrite(ENAdc_pin, lastValue);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
       while (delta*2*degreespertick > 1)
       {
        lastValue-= 1;
        lastPos = curpoint;
        digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, LOW);
        analogWrite(ENAdc_pin, lastValue);
        delay(500);
        delta = abs(curpoint-lastPos);
       }
          analogWrite(ENAdc_pin, startpower);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
       lastValue+=1;
        analogWrite(ENAdc_pin, lastValue);
           lastPos = curpoint;
       delay(500);
        delta = abs(curpoint-lastPos);
       minspeedValue = lastValue;
       minspeed = delta*2*degreespertick;
       #ifdef Debug
       Serial.print("Min speed power value:");
       Serial.println(minspeedValue);
       Serial.print("Min speed value:");
       Serial.println(minspeed);
       #endif
          digitalWrite(IN1dc_pin, HIGH);
       digitalWrite(IN2dc_pin, HIGH);
       analogWrite(ENAdc_pin, 255);
       delay(500);
       digitalWrite(IN1dc_pin, LOW);
       digitalWrite(IN2dc_pin, LOW);
       analogWrite(ENAdc_pin, 0);*/
    //</calibration run>
    EEPROM.update(21, startpower);
    // EEPROM.put(24,speedgain);
    EEPROM.update(22, Direction);
    //  EEPROM.update(23,minspeedValue);
    //  EEPROM.put(32,minspeed);
    //<startpoint set>
#ifdef Debug
    Serial.println("EEPROM updated");
    Serial.println("Shutter position recording initialized!");
#endif
#ifdef Three-Pos
    curpoint = 0;
    startpoint = 0;
    while (!digitalRead(setupbtn_pin))
    {
      delay(5);
    }
    while (digitalRead(setupbtn_pin))
    {
      if (!digitalRead(leftbtn_pin))
      {
        if (Direction > 0)
        {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
        }
        else
        {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
        }
        analogWrite(ENAdc_pin, startpower);
      }
      else if (!digitalRead(rightbtn_pin))
      {
        if (Direction > 0)
        {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
        }
        else
        {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
        }
        analogWrite(ENAdc_pin, startpower);
      }
      else
      { digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, HIGH);

        analogWrite(ENAdc_pin, 255);
      }
      delay(5);
    }
    startpoint = curpoint;
#endif
    //</startpoint set>

    //<midpoint set>
    Move(4);
    midpoint = 0;
    while (!digitalRead(setupbtn_pin))
    {
      delay(5);
    }
    while (digitalRead(setupbtn_pin))
    {
      if (!digitalRead(leftbtn_pin))
      {
        if (Direction > 0)
        {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
        }
        else
        {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
        }
        analogWrite(ENAdc_pin, startpower);
      }
      else if (!digitalRead(rightbtn_pin))
      {
        if (Direction > 0)
        {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
        }
        else
        {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
        }
        analogWrite(ENAdc_pin, startpower);
      }
      else
      { digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, HIGH);

        analogWrite(ENAdc_pin, 255);
      }
      delay(5);
    }
    midpoint = curpoint;
    //</midpoint set>
    //<endpoint set>
    Move(4);
    endpoint = 0;
    while (!digitalRead(setupbtn_pin))
    {
      delay(5);
    }
    while (digitalRead(setupbtn_pin))
    {
      if (!digitalRead(leftbtn_pin))
      {
        if (Direction > 0)
        {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
        }
        else
        {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
        }
        analogWrite(ENAdc_pin, startpower);
      }
      else if (!digitalRead(rightbtn_pin))
      {
        if (Direction > 0)
        {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
        }
        else
        {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
        }
        analogWrite(ENAdc_pin, startpower);
      }
      else
      { digitalWrite(IN1dc_pin, HIGH);
        digitalWrite(IN2dc_pin, HIGH);

        analogWrite(ENAdc_pin, 255);
      }
      delay(5);
    }
    endpoint = curpoint;
    //</endpoint set>
#endif

#ifdef Three-Pos
    EEPROM.put(0, startpoint);
#endif
    EEPROM.put(8, endpoint);
    EEPROM.put(4, midpoint);
    EEPROM.put(12, curpoint);
    EEPROM.update(20, 0);
    Move(1);
    EEPROM.put(12, curpoint);
  }
#ifdef Three-Pos
  EEPROM.get(0, startpoint);
#endif
  EEPROM.get(12, curpoint);
  EEPROM.get(8, endpoint);
  EEPROM.get(4, midpoint);
#ifdef useServo
  servo.write(curpoint);
  delay(1000);
  servo.detach();
#endif
#ifdef useDC
  startpower = EEPROM.read(21);
  //EEPROM.get(24,speedgain);
  Direction = EEPROM.read(22);
  //minspeedValue = EEPROM.read(23);
  //EEPROM.get(32,minspeed);
#endif
}

void loop() {
  short btn = 0;
  if (!digitalRead(rightbtn_pin)) btn = 3;
  if (!digitalRead(leftbtn_pin)) btn = 2;
  if (!digitalRead(setupbtn_pin)) btn = 1;
  switch (btn)
  {
    case 1:
#ifdef Three-Pos
      Move(1);
      EEPROM.put(12, (double)curpoint);
#endif
      break;
    case 2:
      Move(2);
      EEPROM.put(12, (double)curpoint);
      break;
    case 3:
      Move(3);
      EEPROM.put(12, (double)curpoint);
      break;
    default:
      delay(5);
  }
}
///Mode 1 = Close; Mode 2 = Mid; Mode 3 = Open
#ifdef useServo
void Move(int Mode) {
  servo.attach(servo_pin);
  servo.write(curpoint);
  int targetpoint;
  switch (Mode)
  {
    case 1:
#ifdef Three-Pos
      targetpoint = startpoint;
#endif
      break;
    case 2:
      targetpoint = midpoint;
      break;
    case 4:
      targetpoint = 90;
      break;
    default:
      targetpoint = endpoint;
  }
  curspeed = 0;
  short stage = 0;
  while (curpoint != targetpoint)
  {
    int distance = targetpoint - curpoint;
    switch (stage)
    {
      case 0:
        if (abs(distance) > (((double)pow((double)curspeed, 2)) / ((double)acceleration * 2)) / (double)degreespernumber)
        {
          if (curspeed < maxusedspeed)
          {
            curspeed += (double)acceleration / 1000;
            if (distance > 0)
            {
              curpoint += (curspeed / 1000) / (double)degreespernumber;
            }
            else
              curpoint -= (curspeed / 1000) / (double)degreespernumber;
          }
          else {
            stage++;
          }
        }
        else
          stage = 2;
        break;
      case 1:
        if (abs(distance) > (((double)pow((double)maxusedspeed, 2)) / ((double)acceleration * 2)) / (double)degreespernumber)
        {
          if (distance > 0)
            curpoint += ((double)maxusedspeed / 1000) / (double)degreespernumber;
          else
            curpoint -= ((double)maxusedspeed / 1000) / (double)degreespernumber;
        }
        else
          stage++;
        break;
      case 2:
        if (curspeed > 0)
        {
          curspeed -= (double)acceleration / 1000;
          if (distance > 0)
          {
            curpoint += ((double)curspeed / 1000) / (double)degreespernumber;
          }
          else
            curpoint -= ((double)curspeed / 1000) / (double)degreespernumber;
        }
        else
          curpoint = targetpoint;
        break;
    }
    servo.write(curpoint);
#ifdef Debug
    Serial.print(curpoint);
    Serial.println(curspeed);
#endif
    delay(1);
  }
  delay(50);
  servo.detach();
}
#endif

#ifdef useDC
void Move(int Mode) {
  Attach = true;
  switch (Mode)
  {
    case 1:
#ifdef Three-Pos
      targetpoint = startpoint;
#endif
      break;
    case 2:
      targetpoint = midpoint;
      break;
    case 4:
      targetpoint = 0;
      break;
    default:
      targetpoint = endpoint;
  }
  curspeed = 0;
  stage = 0;
  // wantedpoint = curpoint;
  waitStart = -1;
  ServoEmulator();
}
/* void Move(int Mode){
  long targetpoint;
  switch(Mode)
  {
   case 1:
     #ifdef Three-Pos
     targetpoint=startpoint;
     #endif
   break;
   case 2:
     targetpoint=midpoint;
   break;
   case 4:
     targetpoint = 0;
   break;
   default:
     targetpoint=endpoint;
  }
  curspeed = minspeed;
  short stage = 0;
  double distance = (double)targetpoint-curpoint;
  while(abs(distance) > ticktolerance)
  {
   Correct();
   distance = targetpoint-curpoint;
   switch(stage)
   {
  case 0:
  if(abs(distance) > (((double)pow((double)curspeed,2))/((double)acceleration*2)) / (double)degreespertick)
  {
   if(curspeed < maxusedspeed)
   {
   curspeed += (double)acceleration/1000;
   if(distance * Direction > 0)
   {
       digitalWrite(IN1dc_pin, HIGH);
       digitalWrite(IN2dc_pin, LOW);
   }
   else
   {
       digitalWrite(IN1dc_pin, LOW);
       digitalWrite(IN2dc_pin, HIGH);
   }
  //          analogWrite(ENAdc_pin, startpower + (curspeed-startspeed)*speedgain );
   }
   else stage++;
  }
  else
  stage = 2;
  break;
  case 1:
   if(abs(distance) > (((double)pow((double)maxusedspeed,2))/((double)acceleration*2)) / (double)degreespertick)
   {
   if(distance * Direction > 0)
   {
       digitalWrite(IN1dc_pin, HIGH);
       digitalWrite(IN2dc_pin, LOW);
   }
   else
   {
       digitalWrite(IN1dc_pin, LOW);
       digitalWrite(IN2dc_pin, HIGH);
   }
       //analogWrite(ENAdc_pin, maxusedspeedValue);
   }
   else
   stage++;
  break;
  case 2:
   if(curspeed > 0)
   {
    curspeed -= (double)acceleration/1000;
   if(distance * Direction > 0)
   {
       digitalWrite(IN1dc_pin, HIGH);
       digitalWrite(IN2dc_pin, LOW);
   }
   else
   {
       digitalWrite(IN1dc_pin, LOW);
       digitalWrite(IN2dc_pin, HIGH);
   }
      // analogWrite(ENAdc_pin, startpower + (curspeed-startspeed)*speedgain );
   }
   else
   while(abs(distance) > ticktolerance)
   {
     if(distance * Direction > 0)
   {
       digitalWrite(IN1dc_pin, HIGH);
       digitalWrite(IN2dc_pin, LOW);
   }
   else
   {
       digitalWrite(IN1dc_pin, LOW);
       digitalWrite(IN2dc_pin, HIGH);
   }
       analogWrite(ENAdc_pin, startpower);
       distance = (double)targetpoint-curpoint;
   }

  break;
   }
   #ifdef Debug
   Serial.print("Position:");
   Serial.print(curpoint);
   Serial.print(", Speed:");
   Serial.println(curspeed);
   #endif
   delay(1);
  }

  digitalWrite(IN1dc_pin, HIGH);
  digitalWrite(IN2dc_pin, HIGH);
  analogWrite(ENAdc_pin, 255);
  delay(500);
  digitalWrite(IN1dc_pin, LOW);
  digitalWrite(IN2dc_pin, LOW);
  analogWrite(ENAdc_pin, 0);
  }*/
#endif


#ifdef useDC
void EncoderUpdate() {
  if (digitalRead(EncoderA) == HIGH) {
    if (digitalRead(EncoderB) == LOW) {
      curpoint++;
    } else {
      curpoint--;
    }
  } else {
    if (digitalRead(EncoderB) == LOW) {
      curpoint--;
    } else {
      curpoint++;
    }
  }
}
void ServoEmulator() {
  SpeedCalculator();
  unsigned long lastCall = millis();
  while (Attach)
  {
    double distance = targetpoint - curpoint;
    if (distance * Direction > 0)
    {
      digitalWrite(IN1dc_pin, HIGH);
      digitalWrite(IN2dc_pin, LOW);
    }
    else
    {
      digitalWrite(IN1dc_pin, LOW);
      digitalWrite(IN2dc_pin, HIGH);
    }
    analogWrite(ENAdc_pin, startpower + 10);
    if (millis() - lastCall >= 1)
    {
      SpeedCalculator();
      lastCall = millis();
    }
  }
  digitalWrite(IN1dc_pin, HIGH);
  digitalWrite(IN2dc_pin, HIGH);
  analogWrite(ENAdc_pin, 255);
  delay(500);
  digitalWrite(IN1dc_pin, LOW);
  digitalWrite(IN2dc_pin, LOW);
  analogWrite(ENAdc_pin, 0);
}
void SpeedCalculator() {
  if (abs(curpoint - targetpoint) <= ticktolerance)
  {
    if (waitStart == -1)
      waitStart = millis();
    if (millis() - waitStart > 500)
      Attach = false;
  }
#ifdef Debug
  Serial.print(curpoint);
  Serial.print(" ");
  Serial.print(targetpoint);
  Serial.print(" ");
  Serial.println(curspeed);
#endif
}
/*void ServoEmulator() {
  SpeedCalculator();
  unsigned long lastCall = millis();
  while(Attach)
  {
      double distance = wantedpoint-curpoint;
          if(distance * Direction > 0)
      {
          digitalWrite(IN1dc_pin, HIGH);
          digitalWrite(IN2dc_pin, LOW);
      }
      else
      {
          digitalWrite(IN1dc_pin, LOW);
          digitalWrite(IN2dc_pin, HIGH);
      }
          analogWrite(ENAdc_pin, 255);
           if(millis()-lastCall >= 1)
           {
            SpeedCalculator();
            lastCall = millis();
           }
  }
   digitalWrite(IN1dc_pin, HIGH);
   digitalWrite(IN2dc_pin, HIGH);
   analogWrite(ENAdc_pin, 255);
   delay(500);
   digitalWrite(IN1dc_pin, LOW);
   digitalWrite(IN2dc_pin, LOW);
   analogWrite(ENAdc_pin, 0);
  }
  void SpeedCalculator() {
        if (abs(curpoint - targetpoint) <= ticktolerance) Attach = false;
      double distance = targetpoint-curpoint;
      switch(stage)
      {
    case 0:
    if(abs(distance) > (((double)pow((double)curspeed,2))/((double)acceleration*2)) / (double)degreespertick)
    {
      if(curspeed < maxusedspeed)
      {
      curspeed += (double)acceleration/1000;
      if(distance > 0)
      {
      wantedpoint += (curspeed/1000) / (double)degreespertick;
      }
      else
      wantedpoint -= (curspeed/1000) / (double)degreespertick;
      }
      else{
        stage++;
      }
    }
    else
    stage = 2;
    break;
    case 1:
      if(abs(distance) > (((double)pow((double)maxusedspeed,2))/((double)acceleration*2)) / (double)degreespertick)
      {
       if(distance > 0)
          wantedpoint += ((double)maxusedspeed/1000) / (double)degreespertick;
        else
          wantedpoint -= ((double)maxusedspeed/1000) / (double)degreespertick;
      }
      else
      stage++;
    break;
    case 2:
      if(curspeed > 0)
      {
       curspeed -= (double)acceleration/1000;
        if(distance > 0)
        {
          wantedpoint += ((double)curspeed/1000) / (double)degreespertick;
        }
        else
          wantedpoint -= ((double)curspeed/1000) / (double)degreespertick;
      }
      else
       wantedpoint = targetpoint;
    break;
      }
      #ifdef Debug
      Serial.print(curpoint);
      Serial.print(" ");
      Serial.print(wantedpoint);
      Serial.print(" ");
      Serial.println(curspeed);
      #endif
  }*/
/*void Correct()
  {
      if(millis() - last > 50)
    {
     double realSpeed = abs(curpoint-lastpos)*(1000/(millis()-last))*degreespertick;
              lastpos = curpoint;


      if(abs(realSpeed - curspeed) < 25)if(realSpeed-curspeed < 0) curspeedvalue++; else curspeedvalue--;
   else if (abs(realSpeed - curspeed) < 50) if(realSpeed-curspeed < 0) curspeedvalue+= 2; else curspeedvalue-=2;
   else if(realSpeed-curspeed < 0) curspeedvalue+= ((double)acceleration/(double)(1000/(millis()-last)))/speedgain; else curspeedvalue-=((double)acceleration/(double)(1000/(millis()-last)))/speedgain;
         last = millis();

    if(curspeedvalue > 255)
  {
      curspeedvalue = 255;

       analogWrite(ENAdc_pin, curspeedvalue);
  }
  else
    if (curspeedvalue < minspeedValue)
  {
        curspeedvalue = minspeedValue;

    analogWrite(ENAdc_pin, curspeedvalue);
  }
  else
  {
       analogWrite(ENAdc_pin, curspeedvalue);
  }
  }
  }*/
#endif
