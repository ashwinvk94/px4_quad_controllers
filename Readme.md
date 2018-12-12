# px4 controllers
This repository contains controllers which can be used in unison with mavros and the px4 stack in order to achieve minimum snap trajectory tracking.
The position feedback will be done using the vicon system.

**Authors**: Ashwin Varghese Kuruttukulam  
**Maintainer**: Ashwin Varghese Kuruttukulam
**Affiliation**: Perception and Robotics Group, Unversity of Maryland  

## Installation Instructions (Ubuntu)

### Dependencies

1. ROS (http://wiki.ros.org/kinetic/Installation)
2. mavros (https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

1. Install the repository and its dependencies (with rosinstall):

```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

2. Set up a catkin workspace (if not already done):

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd src 
git clone https://github.com/ashwinvk94/px4_quad_controllers
catkin_make
```

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

* Test offboard mode while publishing atitude setpoints

