#!/usr/bin/env python
import numpy
import rospy
import mavros
from geometry_msgs.msg import PoseStamped,Vector3
from mavros_msgs.msg import State,AttitudeTarget,RCIn
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf.transformations
import sys, time

import timeit

from argparse import ArgumentParser

from pyquaternion import Quaternion

import roslaunch
class test:
    def __init__(self):

        self.att_sp_cb_flag = False
        self.thrust_sp_cb_flag = False
        self.rc_cb_flag = False
        self.yaw_sp_cb_flag = False

        #Rate init
        #DECIDE ON PUBLISHING RATE
        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz
        
        self.attitude_thrust_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
        attitude_target_sub = rospy.Subscriber("/px4_quad_controllers/rpy_setpoint", PoseStamped, self.attitude_setpoint_sub_callback)
        thrust_target_sub = rospy.Subscriber("/px4_quad_controllers/thrust_setpoint", PoseStamped, self.thrust_setpoint_sub_callback)
        yaw_target_sub = rospy.Subscriber("/px4_quad_controllers/yaw_setpoint", PoseStamped, self.yaw_setpoint_sub_callback)
        #Manual control sub
        thrust_target_sub = rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_sub_callback)
        
        while not rospy.is_shutdown():
            # if(self.att_sp_cb_flag==True and self.thrust_sp_cb_flag==True):
            
            
            if(self.rc_cb_flag == True and self.att_sp_cb_flag==True and self.yaw_sp_cb_flag==True):
                if(self.thrust_sp_cb_flag==False):
                    self.thrust_sp = rospy.get_param('/attitude_thrust_publisher/thrust_sp')
                #USE THE NEXT 4 LINES ONLY FOR INITIAL TESTING
                #self.att_r = rospy.get_param('/attitude_thrust_publisher/att_r')
                #self.att_p = rospy.get_param('/attitude_thrust_publisher/att_p')
                #self.att_y = rospy.get_param('/attitude_thrust_publisher/att_y')
                #self.thrust_sp = rospy.get_param('/attitude_thrust_publisher/thrust_sp')
                #Manual control
                #self.att_r = self.rc_roll
                #self.att_p = -self.rc_pitch


                # print 'att_r'+str(att_r)
                # print 'att_p'+str(att_p)
                # print 'att_y'+str(att_y)
                # print 'thrust_sp'+str(thrust_sp)
                # print('\n')

                att_quat_w,att_quat_x,att_quat_y,att_quat_z = tf.transformations.quaternion_from_euler(self.att_y,self.att_p,self.att_r, axes='sxyz')
                
                target_attitude_thrust = AttitudeTarget()
                target_attitude_thrust.header.frame_id = "home"
                target_attitude_thrust.header.stamp = rospy.Time.now()
                target_attitude_thrust.type_mask = 7
                target_attitude_thrust.orientation.x = att_quat_x
                target_attitude_thrust.orientation.y = att_quat_y
                target_attitude_thrust.orientation.z = att_quat_z
                target_attitude_thrust.orientation.w = att_quat_w
                target_attitude_thrust.thrust = self.thrust_sp
                self.attitude_thrust_pub.publish(target_attitude_thrust)

            self.rate.sleep()

    def attitude_setpoint_sub_callback(self,state):
        self.att_r = state.pose.position.x
        self.att_p = state.pose.position.y
        #self.att_y = state.pose.position.z
        self.att_sp_cb_flag = True

    def yaw_setpoint_sub_callback(self,state):
        self.att_y = state.pose.position.x
        #self.att_y = state.pose.position.z
        self.yaw_sp_cb_flag = True


    def thrust_setpoint_sub_callback(self,state):
        self.thrust_sp = state.pose.position.x
        self.thrust_sp_cb_flag = True

    def rc_in_sub_callback(self,state):
        thrust_channels = state.channels
        self.rc_roll = (float(thrust_channels[0])-1500)/1000
        self.rc_pitch = (float(thrust_channels[1])-1500)/1000
        self.rc_cb_flag = True

def main(args):
    rospy.init_node('offb_node', anonymous=True)
    ic=test()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
