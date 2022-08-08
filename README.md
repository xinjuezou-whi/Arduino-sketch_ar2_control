# Arduino-sketch_ar2_control

This is the motor controller for AR2 arm. Though AR2 has [its Arduino sketch](https://www.anninrobotics.com/downloads) already, it is not designed for ROS MoveIt. Extending the AR2 to MoveIt application for which is this repository aims.

With a thorough refactor, it can catch up with the frequency of the hardware interface by removing many logics that are redundant to MoveIt. Meanwhile, it keeps the “MJ” protocol as exact as before, which enables communication to “AR2.py” as well.

## Prerequisites
It relies on rosserial library to communicate, so please install such library from library managment of Arduino IDE
![image](https://user-images.githubusercontent.com/72239958/183248539-b3b5ac4b-b4fa-437d-aa0e-2b6feb40bdda.png)


### Potential issue
![image](https://user-images.githubusercontent.com/72239958/183331463-e284af73-4694-451c-803e-ae6c8dfe9612.png)

The compile error: "Rosserial_Arduino_Library\src/ros/msg.h:40:10: fatal error: cstring: No such file or directory" would be prompted, if version 0.9.1 is installed, which is still an open issue in this [thread#518](https://github.com/ros-drivers/rosserial/issues/518).

Degrade the version to 0.7.9 for a temporary solution.

## Wiring
Please check electrical wiring meets the pin’s definition. Two major parts: the direction and pulse of the stepper driver, and the input of the limit switch.
```
const int joint_step_pin_[JOINT_NUM] = { 2, 4, 6, 8, 10, 12 };
const int joint_dir_pin_[JOINT_NUM] = { 3, 5, 7, 9, 11, 13 };
const int joint_limit_pin_[JOINT_NUM] = { 22, 23, 24, 25, 26, 27 };
```

## Usage
## With MoveIt
First make sure the MOVEIT is defined to 1, then compile and upload
```
#define MOVEIT 1
```

Please go to link
https://github.com/xinjuezou-whi/ar2_arm_interface
to refer its usage with MoveIt

## With AR2.py
Change the definition of MOVEIT to 0, then compile and upload

```
#define MOVEIT 0
```

Run the AR2.py. Please note that the Cartesian motion is not implemented so far, only joint movement is valid which means the "Millimeters to Jog" is unfunctional.
