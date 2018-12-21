# px4 controllers
This repository contains controllers which can be used in unison with mavros and the px4 stack in order to achieve minimum snap trajectory tracking.
The position feedback will be done using the vicon system.

**Authors**: Ashwin Varghese Kuruttukulam  
**Maintainer**: Ashwin Varghese Kuruttukulam
**Affiliation**: Perception and Robotics Group, Unversity of Maryland  

## Installation Instructions (Ubuntu)

### Dependencies



1. Install the repository and its dependencies:
* ROS (http://wiki.ros.org/kinetic/Installation)
* mavros (https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

For asll tests, mavros was installed from source. The instructions to install from source in available in the above mentioned link.

2. Set up a catkin workspace (if not already done):

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd src 
git clone https://github.com/ashwinvk94/px4_quad_controllers
catkin_make
```
### TX2 setup Instructions
https://github.com/jetsonhacks/installROSTX2

## Notes
Always start mavros before using any of the functionalities of this repo. On the intel aero drone, this can be done by running the following command on a terminal:

```
roslaunch mavros px4.launch
```

## Usage

To the run the attitude+thrust controller inbuilt in the px4 firmware simply launch attitude_thrust_controller using the command:
'''
roslaunch px4_quad_controllers attitude_thrust_controller.launch
'''

This will start publishing the the attitude and thrust setpoints to the the topic "/mavros/setpoint_raw/attitude"  
Now if you move the flight mode to "OFFBOARD" mode, then it will go that the attitudes and thrust set as per the rosparams. Default values are (0,0,0,0).

## TODO

* Add current yaw update during start of manual contorl in offbaord mdoe
* Create hover thrust control node and test while using manual rc control for roll pitch and yaw
* Test PID controllers for roll and pitch setpoints
* Test offboard mode while publishing atitude setpoints

## Tests completed
* Tested manual flight in offboard mode


