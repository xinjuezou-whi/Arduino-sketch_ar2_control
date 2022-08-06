# Arduino-sketch_ar2_control

This is the motor controller for AR2 arm. Though AR2 has [its Arduino sketch](https://www.anninrobotics.com/downloads) already, it is not designed for ROS MoveIt. Extending the AR2 to MoveIt application for which is this repository aims.

With a thorough refactor, it can catch up with the frequency of the hardware interface by removing many logics that are redundant to MoveIt. Meanwhile, it keeps the “MJ” protocol as exact as before, which enables communication to “AR2.py” as well.

## Prerequisites
It relies on rosserial library to communicate, so please install such library from library managment of Arduino IDE
![image](https://user-images.githubusercontent.com/72239958/183248539-b3b5ac4b-b4fa-437d-aa0e-2b6feb40bdda.png)


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
