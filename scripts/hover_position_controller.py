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

        self.P = rospy.get_param('/attitude_thrust_controller/pos_hover_P')
        self.I = rospy.get_param('/attitude_thrust_controller/pos_hover_I')
        self.D = rospy.get_param('/attitude_thrust_controller/pos_hover_D')
        self.x_pos_pid = PID.PID(self.P, self.I, self.D)
        self.y_pos_pid = PID.PID(self.P, self.I, self.D)
        self.x_pos_pid.setSampleTime(0.01)
        self.y_pos_pid.setSampleTime(0.01)

        self.yaw_sp = rospy.get_param('/attitude_thrust_controller/yaw_sp')

        #Rate init
        #DECIDE ON PUBLISHING RATE
        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz
        
        self.attitude_target_pub = rospy.Publisher("/px4_quad_controllers/rpy_setpoint", PoseStamped, queue_size=10)

        #ADD SUBSCRIBER FOR VICON DATA
        #vicon_sub = rospy.Subscriber("/px4_quad_controllers/thrust_setpoint", Float32, self.vicon_sub_callback)
        
        while not rospy.is_shutdown():
            # if(self.att_sp_cb_flag==True and self.thrust_sp_cb_flag==True):
            self.pos_x_sp = rospy.get_param('/attitude_thrust_controller/pos_x')
            self.pos_y_sp = rospy.get_param('/attitude_thrust_controller/pos_y')

            self.x_pos_pid.SetPoint = self.pos_x_sp
            self.y_pos_pid.SetPoint = self.pos_y_sp

            self.x_pos_pid.update(self.vicon_x)
            self.y_pos_pid.update(self.vicon_y)

            #For this to work, we have to align x,y of quad and vicon
            r_output = self.x_pos_pid.output
            p_output = self.y_pos_pid.output

            target_attitude = PoseStamped()
            target_attitude.header.frame_id = "home"
            target_attitude.header.stamp = rospy.Time.now()
            target_attitude.position.x = r_output
            target_attitude.position.y = p_output
            target_attitude.position.z = yaw_sp

            self.attitude_target_pub.publish(target_attitude)

            self.rate.sleep()

    # def vicon_sub_callback(self,state):
    #     self.att_r = state.x
    #     self.att_p = state.y
    #     self.att_y = state.z
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
