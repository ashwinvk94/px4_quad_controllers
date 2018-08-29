#!/usr/bin/env python
import numpy
import rospy
import mavros
from geometry_msgs.msg import PoseStamped,Vector3
from mavros_msgs.msg import State,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf.transformations
import sys, time
import PID

import timeit

from argparse import ArgumentParser

from pyquaternion import Quaternion

import roslaunch
class test:
    def __init__(self):

        self.vicon_cb_flag = False

        self.P = rospy.get_param('/attitude_thrust_controller/height_hover_P')
        self.I = rospy.get_param('/attitude_thrust_controller/height_hover_I')
        self.D = rospy.get_param('/attitude_thrust_controller/height_hover_D')
        self.height_pid = PID.PID(self.P, self.I, self.D)
        self.height_pid.setSampleTime(0.01)

        #Rate init
        #DECIDE ON PUBLISHING RATE
        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz
        
        self.height_target_pub = rospy.Publisher("/px4_quad_controllers/rpy_setpoint", PoseStamped, queue_size=10)

        #ADD SUBSCRIBER FOR VICON DATA
        #vicon_sub = rospy.Subscriber("/px4_quad_controllers/thrust_setpoint", Float32, self.vicon_sub_callback)
        
        while not rospy.is_shutdown():
            # if(self.vicon_cb_flag==True):
            self.height_sp = rospy.get_param('/attitude_thrust_controller/height_sp')

            self.height_pid.SetPoint = self.height_sp

            self.height_pid.update(self.vicon_height)

            #For this to work, we have to align x,y of quad and vicon
            height_output = self.height_pid.output

            target_height = PoseStamped()
            target_height.header.frame_id = "home"
            target_height.header.stamp = rospy.Time.now()
            target_height.position.x = height_output

            self.height_target_pub.publish(target_height)

            self.rate.sleep()

    # def vicon_sub_callback(self,state):
    #     self.vicon_height = state.z
    #     self.vicon_cb_flag = True

def main(args):
    rospy.init_node('offb_node', anonymous=True)
    ic=test()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
