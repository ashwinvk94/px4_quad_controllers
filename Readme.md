# px4 controllers
This repository contains controllers which can be used in unison with mavros and the px4 stack in order to achieve minimum snap trajectory tracking.
The position feedback will be done using the vicon system.

**Authors**: Ashwin Varghese Kuruttukulam  
**Maintainer**: Ashwin Varghese Kuruttukulam
**Affiliation**: Perception and Robotics Group, Unversity of Maryland  

[//]: # "

## Bibliography
This implementation is largely based on the work of C. Richter *et al*, who should be cited if this is used in a scientific publication (or the preceding conference papers):
[1] C. Richter, A. Bry, and N. Roy, “**Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments,**” in *International Journal of Robotics Research*, Springer, 2016.
```
@incollection{richter2016polynomial,
  title={Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments},
  author={Richter, Charles and Bry, Adam and Roy, Nicholas},
  booktitle={Robotics Research},
  pages={649--666},
  year={2016},
  publisher={Springer}
}
```
"

## Installation Instructions (Ubuntu)
To install this package with [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu):

1. Install additional system dependencies (swap indigo for kinetic as necessary):

```
sudo apt-get install python-wstool python-catkin-tools ros-indigo-cmake-modules libyaml-cpp-dev
```

2. Install the repository and its dependencies (with rosinstall):

```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

3. Set up a catkin workspace (if not already done):

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin_make
```

## Notes
Always start mavros before using any of the functionalities of this repo. On the intel aero drone, this can be done by running the following command on a terminal:
```
rosrun mavros mavros_node _fcu_url:=tcp://127.0.0.1:576_system_:=2
```
Also, you can ssh into the aero drone using the following command:
'''
ssh aero@192.168.8.1
'''
where "Aero" is the user_name of the computer


## Usage

To the run the attitude+thrust controller inbuilt in the px4 firmware simply launch attitude_thrust_controller using the command:
'''
roslaunch px4_quad_controllers attitude_thrust_controller.launch
'''

This will start publishing the the attitude and thrust setpoints to the the topic "/mavros/setpoint_raw/attitude"  
Now if you move the flight mode to "OFFBOARD" mode, then it will go that the attitudes and thrust set as per the rosparams. Default values are (0,0,0,0).
