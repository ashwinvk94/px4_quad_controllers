#!/usr/bin/env python
import numpy
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,PositionTarget,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool
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

        self.local_pose_subscriber_prev=0
        startup_start = timeit.default_timer()
        print('start')

        #Rate init
        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

        self.current_state = State()
        #Current state sub
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_subscriber_callback)

        #Init last_request
        self.last_request = rospy.get_rostime()

        # wait for FCU connection
        print('Waiting for FCU connection...')
        while not self.current_state.connected:
            self.rate.sleep()
        print('Connection sucessful!!')

        startup_stop = timeit.default_timer()
        print('startup time is'+str(startup_stop-startup_start))

        #Quadcopter imu subscriber init
        attitude_target_sub = rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self.attitude_target_sub_callback)
        


    #Current state subscriber
    def state_subscriber_callback(self,state):
        self.current_state = state
        #print(self.current_state)
        self.rate.sleep()

    def attitude_target_sub_callback(self,state):
        orientation = (state.orientation)
        
        #self.ground_wrt_body_quat = Quaternion(orientation.w,orientation.x,orientation.y,orientation.z)
        #quat = state.orientation
        #https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        euler = tf.transformations.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
        print('quad TARGET attitude')
        print('roll = '+ str(euler[2]))
        print('pitch = '+ str(euler[1]))
        print('yaw = '+ str(euler[0]))
        print('\n')
        '''
        self.body_wrt_ground_trans = self.body_wrt_ground_quat.transformation_matrix
        self.ground_wrt_body_trans =numpy.linalg.inv(self.body_wrt_ground_trans)
        print(self.ground_wrt_body_trans)
        '''
        #self.rate.sleep()


def main(args):
    rospy.init_node('offb_node', anonymous=True)
    ic=test()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
