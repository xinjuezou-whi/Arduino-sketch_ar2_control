/******************************************************************
Stepper motor control

Features:
- step motor control
- rosserial communication
- homing mechanism
- xxx

Dependency:
- rosserial
- TimerOne

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-07-09: refactored from version AR2.0
2022-08-02: added homing mechanism
2022-08-11: bit banging with timer interrupt mechanism
2022-08-12: introduce TimerOne to increase the frequency of bit banging
2022-xx-xx: xxx
******************************************************************/
#include <TimerOne.h>
#include <ros.h>
#include <std_msgs/String.h>

#define MOVEIT 1
#define SIMULTANEOUS 1

const String VERSION = "01.11";

// ROS
ros::NodeHandle nh_;
std_msgs::String response_string_;
ros::Publisher pub_("arm_hardware_response", &response_string_);
char pub_data_[64] = { 0 };

// control variables
String recv_string_;
const int JOINT_NUM = 6;
enum STATE { STA_HOMING_LIMIT = 0, STA_HOMING_WAIT, STA_HOMING_ZERO, STA_SLAVE, STA_STANDBY, STA_UNDEFINED };
uint8_t pre_state_ = STA_STANDBY;
uint8_t state_ = STA_STANDBY;
bool bit_banging_toggles_[JOINT_NUM] = { true, true, true, true, true, true };
volatile int request_steps_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
volatile int moving_dir_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
volatile int durations_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
int elapses_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
int homing_index_ = 0;
unsigned long pre_ticks_ = 0;

// status
volatile int cur_pose_[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
// kinematics
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
  // move direction
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
  int lmtStart = Command.indexOf('l');

  // move steps
  memset(Jstep, 0, sizeof(int) * JOINT_NUM);
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    if (jStart[i] > 0)
    {
      int endPos = jStart[i + 1] < 0 || (i == JOINT_NUM - 1) ? spStart : jStart[i + 1]; 
      Jstep[i] = Command.substring(jStart[i] + 2, endPos).toInt();
    }
  }

  // kinematics
  Kinematics[K_SPEED_IN] = Command.substring(spStart + 1, adStart).toInt();
  Kinematics[K_ACC_DUR] = Command.substring(adStart + 1, asStart).toInt();
  Kinematics[K_ACC_SPD] = Command.substring(asStart + 1, ddStart).toInt();
  Kinematics[K_DCC_DUR] = Command.substring(ddStart + 1, dsStart).toInt();
  Kinematics[K_DCC_SPD] = lmtStart < 0 ? Command.substring(dsStart + 1).toInt() : Command.substring(dsStart + 1, lmtStart).toInt();

  // limits direction
  if (lmtStart >= 0)
  {
    for (int i = 0; i < JOINT_NUM; ++i)
    {
      limits_dir_[i] = Command.substring(lmtStart + 1 + i, lmtStart + 2 + i).toInt();
    }
  }
}

void updateInterval(int Percent)
{
  unsigned long interval = (unsigned long)(25000.0 / Percent);
  Timer1.setPeriod(interval);
}

void updateDurations(const int* Jstep)
{
  int longest = 0;
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    longest = Jstep[i] > longest ? Jstep[i] : longest;
  }

  for (int i = 0; i < JOINT_NUM; ++i)
  {
    elapses_[i] = 0;
    durations_[i] = (longest == Jstep[i] || Jstep[i] == 0) ? 1 : (longest - Jstep[i]) / Jstep[i];
    durations_[i] = durations_[i] == 0 ? 1 : durations_[i];
  }
}

void parseMove(const String& Command)
{
  int jDir[JOINT_NUM] = { 0, 0, 0, 0, 0, 0 };
  int kinematics[K_SUM] = { 0, 0, 0, 0, 0 };

  noInterrupts();
  parseKinematics(Command, jDir, request_steps_, kinematics);
  updateDurations(request_steps_);
  updateInterval(kinematics[K_SPEED_IN]);
  interrupts();

  // set the direction
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    moving_dir_[i] = jDir[i];
    digitalWrite(joint_dir_pin_[i], (jDir[i] + 1) % 2);
  }
}

void parseHome(const String& Command)
{
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    elapses_[i] = 0;
    durations_[i] = 1;
  }
  parseKinematics(Command, home_dirs_, home_steps_, home_kinematics_);
  updateInterval(home_kinematics_[K_SPEED_IN]);

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

void parseLimitsDirection(const String& Command)
{
  int lmtStart = Command.indexOf('t');
  // limits direction
  if (lmtStart >= 0)
  {
    for (int i = 0; i < JOINT_NUM; ++i)
    {
      limits_dir_[i] = Command.substring(lmtStart + 1 + i, lmtStart + 2 + i).toInt();
    }
  }
}

void messageCallback(const std_msgs::String& Msg)
{
  // blink the led to indicate the communication
  digitalWrite(output_pin_53_, HIGH - digitalRead(output_pin_53_));

  String cmd = Msg.data;
  String key = cmd.substring(0, 2);
  if (key == "MJ")
  {
    parseMove(cmd);
    
    state_ = STA_SLAVE;
  }
  else if (key == "hm")
  {
    response_string_.data = "homing";
    pub_.publish(&response_string_);

    parseHome(cmd);
    
    state_ = STA_HOMING_LIMIT;
  }
  else if (key == "lt")
  {
    parseLimitsDirection(recv_string_);
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
#if SIMULTANEOUS
        if (elapses_[i] == 0)
        {
#endif
          digitalWrite(joint_step_pin_[i], bit_banging_toggles_[i]);
          bit_banging_toggles_[i] = !bit_banging_toggles_[i];

          if (bit_banging_toggles_[i])
          {
            --request_steps_[i];
            cur_pose_[i] = moving_dir_[i] == 0 ? cur_pose_[i] - 1 : cur_pose_[i] + 1;
          }
#if SIMULTANEOUS
        }
        elapses_[i] = (elapses_[i] + 1) % durations_[i];
#endif
      }
    }
  }
}

void jointPoses()
{
  pub_data_[0] = 'p';
  pub_data_[1] = 0;
  // to read a variable which the interrupt code writes,
  // we must temporarily disable interrupts, to be sure it will not change while we are reading
  // to minimize the time with interrupts off, just quickly make a copy
  noInterrupts();
  for (int i = 0; i < JOINT_NUM; ++i)
  {
    strcat(pub_data_, String(cur_pose_[i]).c_str());
    strcat(pub_data_, "p");
  }
  interrupts();

#if MOVEIT
  response_string_.data = pub_data_;
  pub_.publish(&response_string_);
#else
  Serial.println(pub_data_);
#endif
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

  Timer1.initialize(1000); // default 1ms
  Timer1.attachInterrupt(driveJoints);
}

void loop()
{
  // report the position of joints
  auto curTicks = millis();
  if (curTicks - pre_ticks_ > 20) // default 20ms
  {
    jointPoses();
    
    pre_ticks_ = curTicks;
  }

  // state machine
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
    {
      noInterrupts();
      auto steps = request_steps_[homing_index_];
      interrupts();
      if (steps == 0)
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
#if !MOVEIT
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
        else if (function == "lt")
        {
          Serial.println("limits received");
          parseLimitsDirection(recv_string_);
        }

        recv_string_ = ""; // clear recieved buffer
      }
    }
#endif
    break;
  }

#if MOVEIT
  // ROS spinner
  nh_.spinOnce();
#endif
}
