/******************************************************************
Stepper motor control

Features:
- step motor control
- rosserial communication
- homing mechanism
- xxx

Dependency:
- rosserial
- Timerinterrupt

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-09: refactored from version AR2.0
2022-08-02: added homing mechanism
2022-08-11: bit banging with timer interrupt mechanism
2022-xx-xx: xxx
******************************************************************/
// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_1     true

#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     true
  #warning Using Timer1, Timer2
#else          
  #define USE_TIMER_3     true
  #warning Using Timer1, Timer3
#endif

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "TimerInterrupt.h"

#include <ros.h>
#include <std_msgs/String.h>

#define MOVEIT 1

const String VERSION = "01.09";

// ROS
ros::NodeHandle nh_;
std_msgs::String response_string_;
ros::Publisher pub_("arm_hardware_response", &response_string_);
char pub_data_[32] = { 0 };

// control variables
String recv_string_;
const int JOINT_NUM = 6;
enum STATE { STA_HOMING_LIMIT = 0, STA_HOMING_WAIT, STA_HOMING_ZERO, STA_SLAVE, STA_STANDBY, STA_UNDEFINED };
uint8_t pre_state_ = STA_STANDBY;
uint8_t state_ = STA_STANDBY;
long pre_ticks_ = 0;
long last_home_ticks_ = 0;
bool bit_banging_toggles_[JOINT_NUM] = { true, true, true, true, true, true };
volatile int request_steps_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
volatile int moving_dir_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
int homing_index_ = 0;

// status
volatile int cur_pose_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };

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

void parseMove(const String& Command)
{
  int jDir[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
  int kinematics[K_SUM] = { 0, 0, 0, 0, 0 };
  parseKinematics(Command, jDir, request_steps_, kinematics);
#if MOVEIT
#else
  int interval = int(100.0 / kinematics[K_SPEED_IN]);
  ITimer1.setInterval(interval, driveJoints);
#endif

  // set the direction
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    moving_dir_[i] = jDir[i];
    digitalWrite(joint_dir_pin_[i], (jDir[i] + 1) % 2);
  }
}

void parseHome(const String& Command)
{
  parseKinematics(Command, home_dirs_, home_steps_, home_kinematics_);
#if MOVEIT
#else
  int interval = int(100.0 / home_kinematics_[K_SPEED_IN]);
  ITimer1.setInterval(interval, driveJoints);
#endif

  // infer the direction of limits
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    limits_dir_[i] = (home_dirs_[i] + 1) % 2;
  }
  // get the index of homing-active joint
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    if (home_steps_[i] > 0)
    {
      homing_index_ = i;
      break;
    }
  }  
}

void messageCallback(const std_msgs::String& Msg)
{
  digitalWrite(53, HIGH - digitalRead(53)); // blink the led

  String cmd = Msg.data;
  state_ = cmd.substring(0, 2) == "hm" ? STA_HOMING_LIMIT : STA_SLAVE;

  if (state_ == STA_SLAVE)
  {
    parseMove(cmd);
  }
  else if (state_ == STA_HOMING_LIMIT)
  {
    response_string_.data = "homing";
    pub_.publish(&response_string_);

    parseHome(cmd);
  }
}

ros::Subscriber<std_msgs::String> sub_("arm_hardware_interface", &messageCallback);

void driveJoints()
{
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    if (request_steps_[i] > 0)
    {
      if (moving_dir_[i] == limits_dir_[i] && digitalRead(joint_limit_pin_[i]) == LOW)
      {
        request_steps_[i] = 0;
      }
      else
      {
        digitalWrite(joint_step_pin_[i], bit_banging_toggles_[i]);
        bit_banging_toggles_[i] = !bit_banging_toggles_[i];

        if (bit_banging_toggles_[i])
        {
          --request_steps_[i];
          cur_pose_[i] = moving_dir_[i] == 0 ? cur_pose_[i] - 1 : cur_pose_[i] + 1;
        }
      }
    }
  }
}

void jointPoses()
{
  for (int i = 0; i < JOINT_NUM; ++i)
  {
#if MOVEIT
    sprintf(pub_data_, "p%d%d", i, cur_pose_[i]);
    response_string_.data = pub_data_;
    pub_.publish(&response_string_);
#else
    Serial.println(String(i) + String(" ") + String(cur_pose_[i]));
#endif
  }
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

  ITimer1.init();
  if (ITimer1.attachInterruptInterval(1, driveJoints))
  {
    Serial.print(F("Starting  ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  else
  {
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
  }
  
  ITimer3.init();
  if (ITimer3.attachInterruptInterval(20, jointPoses))
  {
    Serial.print(F("Starting  ITimer3 OK, millis() = ")); Serial.println(millis());
  }
  else
  {
    Serial.println(F("Can't set ITimer3. Select another freq. or timer"));
  }
}

void loop()
{
  switch (state_)
  {
  case STA_HOMING_LIMIT:
    if (home_steps_[homing_index_] > 0)
    {
      // set the direction
      moving_dir_[homing_index_] = limits_dir_[homing_index_];
      digitalWrite(joint_dir_pin_[homing_index_], (limits_dir_[homing_index_] + 1) % 2);
      request_steps_[homing_index_] = 20000;

      pre_state_ = state_;
      state_ = STA_HOMING_WAIT;
    }
    else
    {
      state_ = STA_HOMING_ZERO;
    }
    break;
  case STA_HOMING_WAIT:
    if (request_steps_[homing_index_] == 0)
    {
      if (pre_state_ == STA_HOMING_LIMIT)
      {
        state_ = STA_HOMING_ZERO;
      }
      else if (pre_state_ == STA_HOMING_ZERO)
      {
        cur_pose_[homing_index_] = home_steps_[homing_index_] == 0 ? cur_pose_[homing_index_] : 0;
        
        if (++homing_index_ == JOINT_NUM)
        {
          state_ = STA_STANDBY;
#if MOVEIT
          response_string_.data = "homed";
          pub_.publish(&response_string_);
#endif
        }
        else
        {
          state_ = STA_HOMING_LIMIT;
        }
      }
    }
    break;
  case STA_HOMING_ZERO:
    // set the direction
    moving_dir_[homing_index_] = home_dirs_[homing_index_];
    digitalWrite(joint_dir_pin_[homing_index_], (home_dirs_[homing_index_] + 1) % 2);
    request_steps_[homing_index_] = home_steps_[homing_index_];

    pre_state_ = state_;
    state_ = STA_HOMING_WAIT;
    break;
  default:
#if MOVEIT
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
          parseMove(recv_string_);
        }
        else if (function == "hm")
        {
          Serial.println("homing received");
          parseHome(recv_string_);
          
          state_ = STA_HOMING_LIMIT;
        }

        recv_string_ = ""; // clear recieved buffer
      }
    }
#endif
    break;
  }

#if MOVEIT
  nh_.spinOnce();
#endif
}
