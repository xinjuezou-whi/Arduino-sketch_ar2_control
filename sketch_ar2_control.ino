/******************************************************************
Stepper motor control

Features:
- step motor control
- rosserial communication
- homing mechanism
- xxx

Dependency:
- rosserial

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-09: refactored from version AR2.0
2022-08-02: added homing mechanism
2022-xx-xx: xxx
******************************************************************/
#include <ros.h>
#include <std_msgs/String.h>

#define MOVEIT 1

const String VERSION = "00.09";

// ROS
ros::NodeHandle nh_;
std_msgs::String response_string_;
ros::Publisher pub_("arm_hardware_response", &response_string_);
char pub_data_[32] = { 0 };

// control variables
String recv_string_;
String cmd_;
const int JOINT_NUM = 6;
enum STATE { STA_HOMING = 0, STA_SLAVE, STA_STANDBY, STA_UNDEFINED };
uint8_t state_ = STA_SLAVE;
long pre_ticks_ = 0;
long last_home_ticks_ = 0;

// status
int cur_pose_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };

// SPEED // millisecond multiplier // raise value to slow robot speeds // DEFAULT = 200
const int SPEED_MULTIPLE = 200;
enum KINEMATICS { K_SPEED_IN = 0, K_ACC_DUR, K_ACC_SPD, K_DCC_DUR, K_DCC_SPD, K_SUM };
int home_dirs_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
int limits_dir_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
int home_steps_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
int home_kinematics_[K_SUM] = { 0, 0, 0, 0, 0 };

// IO
const int joint_step_pin_[JOINT_NUM] = { 2, 4, 6, 8, 10, 12 };
const int joint_dir_pin_[JOINT_NUM] = { 3, 5, 7, 9, 11, 13 };
const int joint_limit_pin_[JOINT_NUM] = { 22, 23, 24, 25, 26, 27 };
const int output_pin_53_ = 53;

void parseKinematics(const String& Command, int* Jdir, int* Jstep, int* Kinematics)
{
  memset(Jdir, 0, sizeof(int) * JOINT_NUM);
  int jStart[JOINT_NUM];
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    jStart[i] = Command.indexOf(65 + i);
    Jdir[i] = Command.substring(jStart[i] + 1, jStart[i] + 2).toInt();
  }

  int spStart = Command.indexOf('S');
  int adStart = Command.indexOf('G');
  int asStart = Command.indexOf('H');
  int ddStart = Command.indexOf('I');
  int dsStart = Command.indexOf('K');

  memset(Jstep, 0, sizeof(int) * JOINT_NUM);
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    if (jStart[i] > 0)
    {
      int endPos = jStart[i + 1] < 0 || (i == JOINT_NUM - 1) ? spStart : jStart[i + 1]; 
      Jstep[i] = Command.substring(jStart[i] + 2, endPos).toInt();
    }
  }
  
  Kinematics[K_SPEED_IN] = Command.substring(spStart + 1, adStart).toInt();
  Kinematics[K_ACC_DUR] = Command.substring(adStart + 1, asStart).toInt();
  Kinematics[K_ACC_SPD] = Command.substring(asStart + 1, ddStart).toInt();
  Kinematics[K_DCC_DUR] = Command.substring(ddStart + 1, dsStart).toInt();
  Kinematics[K_DCC_SPD] = Command.substring(dsStart + 1).toInt();
}

// DRIVE MOTORS J
void driveMotorsJ(const String& Command)
{
  int jDir[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
  int jStep[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
  int kinematics[K_SUM] = { 0, 0, 0, 0, 0 };
  parseKinematics(Command, jDir, jStep, kinematics);

  // FIND HIGHEST STEP
  int highStep = 0;
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    if (jStep[i] > highStep)
    {
      highStep = jStep[i];
    }
  }

  // FIND ACTIVE JOINTS
  int jActive = 0;
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    jActive += jStep[i] >= 1 ? 1 : 0;
  }

  int jCur[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
  
  int curHighStep = 0;
  int curDelay = 0;

  // SET DIRECTIONS
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    digitalWrite(joint_dir_pin_[i], (jDir[i] + 1) % 2);
  }

  ///// CALC SPEEDS //////
  // REG SPEED
  float speedRatio = kinematics[K_SPEED_IN] / 100.0;
  int regSpeed = int(SPEED_MULTIPLE / speedRatio);
  
  // ACC SPEED
  int accStep = kinematics[K_ACC_DUR] == 0 ? 0 : int(highStep * (kinematics[K_ACC_DUR] / 100.0));
  float accSpdT = kinematics[K_ACC_SPD] / 100.0;
  float accSpeed = ((SPEED_MULTIPLE + (SPEED_MULTIPLE / accSpdT)) / speedRatio);
  int accInc = accStep == 0 ? 0 : (regSpeed - accSpeed) / accStep;

  // DCC SPEED
  int dccStep = kinematics[K_DCC_DUR] == 0 ? 0 : int(highStep - (highStep * (kinematics[K_DCC_DUR] / 100.0)));
  float dccSpdT = kinematics[K_DCC_SPD] / 100.0;
  float dccSpeed = ((SPEED_MULTIPLE + (SPEED_MULTIPLE / dccSpdT)) / speedRatio);
  int dccInc = dccStep == 0 ? 0 : (regSpeed + dccSpeed) / dccStep;

  ///// DRIVE MOTORS /////
  while (jStep[0] > 0 || jStep[1] > 0 || jStep[2] > 0 || jStep[3] > 0 || jStep[4] > 0 || jStep[5] > 0)
  {    
    //// DELAY CALC /////
    if (accStep > 0 && curHighStep <= accStep)
    {
      curDelay = int(accSpeed / jActive);
      accSpeed = accSpeed + accInc;
    }
    else if (dccStep > 0 && curHighStep >= dccStep)
    {
      curDelay = abs(int(dccSpeed / jActive));
      dccSpeed = dccSpeed + dccInc;
    }
    else
    {
      curDelay = int(regSpeed / jActive);
    }

    // joint pulse
    for (int i = 0; i < JOINT_NUM; ++i)
    {
        if (jStep[i] > 0)
        {
          if (jDir[i] == limits_dir_[i] && digitalRead(joint_limit_pin_[i]) == LOW)
          {
            digitalWrite(joint_step_pin_[i], HIGH);
            jStep[i] = 0;
#ifdef DEBUG
            Serial.println("limit triggered");
#endif
          }
          else
          {
            ++jCur[i];
            --jStep[i];
            digitalWrite(joint_step_pin_[i], LOW);
            delayMicroseconds(curDelay);
            digitalWrite(joint_step_pin_[i], HIGH);

            long cur = millis();
            if (cur - pre_ticks_ > 1)
            {
              int pose = cur_pose_[i] + (jDir[i] == 0 ? -jCur[i] : jCur[i]);
#if MOVEIT
              sprintf(pub_data_, "p%d%d", i, pose);
              response_string_.data = pub_data_;
              pub_.publish(&response_string_);
#else
#ifdef DEBUG
              Serial.print("cur ");
              Serial.println(jCur[i]);
              Serial.println(pose);
#endif
#endif
              pre_ticks_ = cur;
            }
          }
        }
    }

    // increase cur step
    ++curHighStep;
  }

  // update current position
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    cur_pose_[i] += jCur[i] * (jDir[i] == 0 ? -1 : 1);
  }

  state_ = STA_STANDBY;
}

void messageCallback(const std_msgs::String& Msg)
{
  digitalWrite(53, HIGH - digitalRead(53)); // blink the led

  cmd_ = Msg.data;
  state_ = cmd_.substring(0, 2) == "hm" ? STA_HOMING : STA_SLAVE;
  
  if (state_ == STA_HOMING)
  {
    response_string_.data = "homing";
    pub_.publish(&response_string_);
    parseKinematics(cmd_, home_dirs_, home_steps_, home_kinematics_);
    // infer the direction of limits
    for (int i =0; i < JOINT_NUM; ++i)
    {
      limits_dir_[i] = (home_dirs_[i] + 1) % 2;
    }
  }
}

ros::Subscriber<std_msgs::String> sub_("arm_hardware_interface", &messageCallback);

void homing()
{
  if (millis() - last_home_ticks_ > 1000)
  {
    for (int i = 0; i < JOINT_NUM; ++i)
    {
      if (home_steps_[i] > 0)
      {
        // rotate towards limit switch
        String cmd = "MJ" + String(char(65 + i)) + String(limits_dir_[i]) + "20000S20G15H15I15K15";
        driveMotorsJ(cmd);
        // back to home position
        cmd = "MJ" + String(char(65 + i)) + String(home_dirs_[i]) + String(home_steps_[i]) + 
          "S" + String(home_kinematics_[K_SPEED_IN]) + 
          "G" + String(home_kinematics_[K_ACC_DUR]) + "H" + String(home_kinematics_[K_ACC_SPD]) +
          "I" + String(home_kinematics_[K_DCC_DUR]) + "K" + String(home_kinematics_[K_DCC_SPD]);

        // rotate towards home position
        driveMotorsJ(cmd);

        cur_pose_[i] = 0;
#if MOVEIT
        sprintf(pub_data_, "p%d%d", i, cur_pose_[i]);
        response_string_.data = pub_data_;
        pub_.publish(&response_string_);
#endif
      }
    }

#if MOVEIT
    response_string_.data = "homed";
    pub_.publish(&response_string_);
#endif

    last_home_ticks_ = millis();
  }

  state_ = STA_STANDBY;
}

void setup()
{
#if MOVEIT
  // init node
  nh_.getHardware()->setBaud(115200);
  nh_.initNode();
  nh_.subscribe(sub_);
  nh_.advertise(pub_);
#else
  Serial.begin(115200);
  Serial.println("starting up...");
#endif

  for (int i = 0; i < JOINT_NUM; ++i)
  {
    pinMode(joint_step_pin_[i], OUTPUT);
    pinMode(joint_dir_pin_[i], OUTPUT);
    pinMode(joint_limit_pin_[i], INPUT_PULLUP);

    digitalWrite(joint_step_pin_[i], HIGH);
  }
  pinMode(output_pin_53_, OUTPUT); // LED checking
}

void loop()
{
#if MOVEIT
  switch (state_)
  {
  case STA_SLAVE:
    driveMotorsJ(cmd_);
    break;
  case STA_HOMING:
    homing();
    break;
  default:
    break;
  }

  nh_.spinOnce();
#else
  // for responding python control sw
  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    recv_string_ += recieved;
    if (recieved == '\n')
    {
      String function = recv_string_.substring(0, 2);
      if (function == "MJ")
      {
        Serial.print("command recieved");
        driveMotorsJ(recv_string_);
      }
      else if (function == "hm")
      {
        Serial.println("homing received");
        parseKinematics(recv_string_, home_dirs_, home_steps_, home_kinematics_);
        homing();
      }

      recv_string_ = ""; // clear recieved buffer
    }
  }
#endif
}
