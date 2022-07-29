/******************************************************************
Stepper motor control

Features:
- step motor control
- rosserial communication
- xxx

Dependency:
- rosserial

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-09: refactored from version AR2.0
2022-xx-xx: xxx
******************************************************************/
#include <ros.h>
#include <std_msgs/String.h>

#define MOVEIT 0

// ROS
ros::NodeHandle nh;

// SPEED // millisecond multiplier // raise value to slow robot speeds // DEFAULT = 200
const int SpeedMult = 200;

/*
  MOTOR DIRECTION - motor directions can be changed on the caibration page in the software but can also
  be changed here: example using DM542T driver(CW) set to 1 - if using ST6600 or DM320T driver(CCW) set to 0
  DEFAULT = 111011   */

const int J1rotdir = 1;
const int J2rotdir = 1;
const int J3rotdir = 1;
const int J4rotdir = 0;
const int J5rotdir = 1;
const int J6rotdir = 1;

const int TRACKrotdir = 0;

String inData;
String function;
char WayPt[101][50];
int WayPtDel;

const int J1stepPin = 2; // 3 for modified
const int J1dirPin = 3; // 2 for modified
const int J2stepPin = 4;
const int J2dirPin = 5; // 2 for modified
const int J3stepPin = 6;
const int J3dirPin = 7;
const int J4stepPin = 8;
const int J4dirPin = 9;
const int J5stepPin = 10;
const int J5dirPin = 11;
const int J6stepPin = 12;
const int J6dirPin = 13;
const int TRstepPin = 20;
const int TRdirPin = 21;

const int J1calPin = 14;
const int J2calPin = 15;
const int J3calPin = 16;
const int J4calPin = 17;
const int J5calPin = 18;
const int J6calPin = 19;

const int Input22 = 22;
const int Input23 = 23;
const int Input24 = 24;
const int Input25 = 25;
const int Input26 = 26;
const int Input27 = 27;
const int Input28 = 28;
const int Input29 = 29;
const int Input30 = 30;
const int Input31 = 31;
const int Input32 = 32;
const int Input33 = 33;
const int Input34 = 34;
const int Input35 = 35;
const int Input36 = 36;
const int Input37 = 37;

const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;
const int Output42 = 42;
const int Output43 = 43;
const int Output44 = 44;
const int Output45 = 45;
const int Output46 = 46;
const int Output47 = 47;
const int Output48 = 48;
const int Output49 = 49;
const int Output50 = 50;
const int Output51 = 51;
const int Output52 = 52;
const int Output53 = 53;

String pre_cmd_;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void driveMotorsJ(String inData)
{
  int J1start = inData.indexOf('A');
  int J2start = inData.indexOf('B');
  int J3start = inData.indexOf('C');
  int J4start = inData.indexOf('D');
  int J5start = inData.indexOf('E');
  int J6start = inData.indexOf('F');
  int TRstart = inData.indexOf('T');
  int Adstart = inData.indexOf('G');
  int Asstart = inData.indexOf('H');
  int Ddstart = inData.indexOf('I');
  int Dsstart = inData.indexOf('K');
  int SPstart = inData.indexOf('S');
  int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
  int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
  int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
  int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
  int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
  int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
  int TRdir = inData.substring(TRstart + 1, TRstart + 2).toInt();
  int J1step = inData.substring(J1start + 2, J2start).toInt();
  int J2step = inData.substring(J2start + 2, J3start).toInt();
  int J3step = inData.substring(J3start + 2, J4start).toInt();
  int J4step = inData.substring(J4start + 2, J5start).toInt();
  int J5step = inData.substring(J5start + 2, J6start).toInt();
  int J6step = inData.substring(J6start + 2, TRstart).toInt();
  int TRstep = inData.substring(TRstart + 2, SPstart).toInt();
  float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();
  float ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
  float ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
  float DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
  float DCCspd = inData.substring(Dsstart + 1).toInt();

  // FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (TRstep > HighStep)
  {
    HighStep = TRstep;
  }

  // FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int TRactive = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (TRstep >= 1)
  {
    TRactive = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int TR_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int TR_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int TR_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int TR_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int TR_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int TRcur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int TR_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int TR_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int TR_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;


  //SET DIRECTIONS

  /////// J1 /////////
  if (J1dir == 1 && J1rotdir == 1)
  {
    digitalWrite(J1dirPin, LOW);
  }
  else if (J1dir == 1 && J1rotdir == 0)
  {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1dir == 0 && J1rotdir == 1)
  {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1dir == 0 && J1rotdir == 0)
  {
    digitalWrite(J1dirPin, LOW);
  }

  /////// J2 /////////
  if (J2dir == 1 && J2rotdir == 1)
  {
    digitalWrite(J2dirPin, LOW);
  }
  else if (J2dir == 1 && J2rotdir == 0)
  {
    digitalWrite(J2dirPin, HIGH);
  }
  else if (J2dir == 0 && J2rotdir == 1)
  {
    digitalWrite(J2dirPin, HIGH);
  }
  else if (J2dir == 0 && J2rotdir == 0)
  {
    digitalWrite(J2dirPin, LOW);
  }

  /////// J3 /////////
  if (J3dir == 1 && J3rotdir == 1)
  {
    digitalWrite(J3dirPin, LOW);
  }
  else if (J3dir == 1 && J3rotdir == 0)
  {
    digitalWrite(J3dirPin, HIGH);
  }
  else if (J3dir == 0 && J3rotdir == 1)
  {
    digitalWrite(J3dirPin, HIGH);
  }
  else if (J3dir == 0 && J3rotdir == 0)
  {
    digitalWrite(J3dirPin, LOW);
  }

  /////// J4 /////////
  if (J4dir == 1 && J4rotdir == 1)
  {
    digitalWrite(J4dirPin, LOW);
  }
  else if (J4dir == 1 && J4rotdir == 0)
  {
    digitalWrite(J4dirPin, HIGH);
  }
  else if (J4dir == 0 && J4rotdir == 1)
  {
    digitalWrite(J4dirPin, HIGH);
  }
  else if (J4dir == 0 && J4rotdir == 0)
  {
    digitalWrite(J4dirPin, LOW);
  }

  /////// J5 /////////
  if (J5dir == 1 && J5rotdir == 1)
  {
    digitalWrite(J5dirPin, LOW);
  }
  else if (J5dir == 1 && J5rotdir == 0)
  {
    digitalWrite(J5dirPin, HIGH);
  }
  else if (J5dir == 0 && J5rotdir == 1)
  {
    digitalWrite(J5dirPin, HIGH);
  }
  else if (J5dir == 0 && J5rotdir == 0)
  {
    digitalWrite(J5dirPin, LOW);
  }

  /////// J6 /////////
  if (J6dir == 1 && J6rotdir == 1)
  {
    digitalWrite(J6dirPin, LOW);
  }
  else if (J6dir == 1 && J6rotdir == 0)
  {
    digitalWrite(J6dirPin, HIGH);
  }
  else if (J6dir == 0 && J6rotdir == 1)
  {
    digitalWrite(J6dirPin, HIGH);
  }
  else if (J6dir == 0 && J6rotdir == 0)
  {
    digitalWrite(J6dirPin, LOW);
  }

  /////// TRACK /////////
  if (TRdir == 1 && TRACKrotdir == 1)
  {
    digitalWrite(TRdirPin, LOW);
  }
  else if (TRdir == 1 && TRACKrotdir == 0)
  {
    digitalWrite(TRdirPin, HIGH);
  }
  else if (TRdir == 0 && TRACKrotdir == 1)
  {
    digitalWrite(TRdirPin, HIGH);
  }
  else if (TRdir == 0 && TRACKrotdir == 0)
  {
    digitalWrite(TRdirPin, LOW);
  }

  /////CALC SPEEDS//////
  float ACCStep = (HighStep * (ACCdur / 100));
  float DCCStep = HighStep - (HighStep * (DCCdur / 100));
  float AdjSpeed = (SpeedIn / 100);
  // REG SPEED
  float CalcRegSpeed = (SpeedMult / AdjSpeed);
  int REGSpeed = int(CalcRegSpeed);

  // ACC SPEED
  float ACCspdT = (ACCspd / 100);
  float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
  float ACCSpeed = (CalcACCSpeed);
  float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

  // DCC SPEED
  float DCCspdT = (DCCspd / 100);
  float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
  float DCCSpeed = (CalcDCCSpeed);
  float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
  DCCSpeed = REGSpeed;

  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
  {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep)
    {
      curDelay = (ACCSpeed / Jactive);
      ACCSpeed = ACCSpeed + ACCinc;
    }
    else if (highStepCur >= DCCStep)
    {
      curDelay = (DCCSpeed / Jactive);
      DCCSpeed = DCCSpeed + DCCinc;
    }
    else
    {
      curDelay = (REGSpeed / Jactive);
    }

    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J1stepPin, HIGH);
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }

    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J2stepPin, HIGH);
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J3stepPin, HIGH);
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }

    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J4stepPin, HIGH);
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }

    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J5stepPin, HIGH);
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }

    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J6stepPin, HIGH);
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }

    /////// TR ////////////////////////////////
    ///find pulse every
    if (TRcur < TRstep)
    {
      TR_PE = (HighStep / TRstep);
      ///find left over 1
      TR_LO_1 = (HighStep - (TRstep * TR_PE));
      ///find skip 1
      if (TR_LO_1 > 0)
      {
        TR_SE_1 = (HighStep / TR_LO_1);
      }
      else
      {
        TR_SE_1 = 0;
      }
      ///find left over 2
      if (TR_SE_1 > 0)
      {
        TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
      }
      else
      {
        TR_LO_2 = 0;
      }
      ///find skip 2
      if (TR_LO_2 > 0)
      {
        TR_SE_2 = (HighStep / TR_LO_2);
      }
      else
      {
        TR_SE_2 = 0;
      }
      /////////  TR  ///////////////
      if (TR_SE_2 == 0)
      {
        TR_SE_2cur = (TR_SE_2 + 1);
      }
      if (TR_SE_2cur != TR_SE_2)
      {
        TR_SE_2cur = ++TR_SE_2cur;
        if (TR_SE_1 == 0)
        {
          TR_SE_1cur = (TR_SE_1 + 1);
        }
        if (TR_SE_1cur != TR_SE_1)
        {
          TR_SE_1cur = ++TR_SE_1cur;
          TR_PEcur = ++TR_PEcur;
          if (TR_PEcur == TR_PE)
          {
            TRcur = ++TRcur;
            TR_PEcur = 0;
            digitalWrite(TRstepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(TRstepPin, HIGH);
          }
        }
        else
        {
          TR_SE_1cur = 0;
        }
      }
      else
      {
        TR_SE_2cur = 0;
      }
    }

    // inc cur step
    highStepCur = ++highStepCur;
  }
}

void messageCallback(const std_msgs::String& toggle_msg)
{
  digitalWrite(13, HIGH-digitalRead(13)); // blink the led
  digitalWrite(53, HIGH-digitalRead(53));
  
  String cmd(toggle_msg.data);
  if (cmd != pre_cmd_)
  {
    driveMotorsJ(cmd);
    
    pre_cmd_ = cmd;
  }
}

ros::Subscriber<std_msgs::String> sub("arm_hardware_interface", &messageCallback);

void setup()
{
  Serial.begin(115200);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(TRstepPin, OUTPUT);
  pinMode(TRdirPin, OUTPUT);
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);

  pinMode(J1calPin, INPUT_PULLUP);
  pinMode(J2calPin, INPUT_PULLUP);
  pinMode(J3calPin, INPUT_PULLUP);
  pinMode(J4calPin, INPUT_PULLUP);
  pinMode(J5calPin, INPUT_PULLUP);
  pinMode(J6calPin, INPUT_PULLUP);

  pinMode(Input22, INPUT_PULLUP);
  pinMode(Input23, INPUT_PULLUP);
  pinMode(Input24, INPUT_PULLUP);
  pinMode(Input25, INPUT_PULLUP);
  pinMode(Input26, INPUT_PULLUP);
  pinMode(Input27, INPUT_PULLUP);
  pinMode(Input28, INPUT_PULLUP);
  pinMode(Input29, INPUT_PULLUP);
  pinMode(Input30, INPUT_PULLUP);
  pinMode(Input31, INPUT_PULLUP);
  pinMode(Input32, INPUT_PULLUP);
  pinMode(Input33, INPUT_PULLUP);
  pinMode(Input34, INPUT_PULLUP);
  pinMode(Input35, INPUT_PULLUP);
  pinMode(Input36, INPUT_PULLUP);
  pinMode(Input37, INPUT_PULLUP);

  pinMode(Output38, OUTPUT);
  pinMode(Output39, OUTPUT);
  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);
  pinMode(Output42, OUTPUT);
  pinMode(Output43, OUTPUT);
  pinMode(Output44, OUTPUT);
  pinMode(Output45, OUTPUT);
  pinMode(Output46, OUTPUT);
  pinMode(Output47, OUTPUT);
  pinMode(Output48, OUTPUT);
  pinMode(Output49, OUTPUT);
  pinMode(Output50, OUTPUT);
  pinMode(Output51, OUTPUT);
  pinMode(Output52, OUTPUT);
  pinMode(Output53, OUTPUT); // commented for modified

  digitalWrite(Output38, HIGH);
  digitalWrite(Output39, HIGH);
  digitalWrite(Output40, HIGH);
  digitalWrite(Output41, HIGH);
  digitalWrite(Output42, HIGH);
  digitalWrite(Output43, HIGH);
  digitalWrite(Output44, HIGH);
  digitalWrite(Output45, HIGH);

  digitalWrite(TRstepPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);

#if MOVEIT
  // init node
  nh.initNode();
  nh.subscribe(sub);
#endif
}

void loop()
{
#if MOVEIT
  nh.spinOnce();
  delay(1);
#else
  // for responding python control sw
  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    inData += recieved;
    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);

      if (function == "MJ")
      {
        Serial.print("command recieved");
        driveMotorsJ(inData);
        inData = ""; // Clear recieved buffer
      }
    }
  }
#endif
}
