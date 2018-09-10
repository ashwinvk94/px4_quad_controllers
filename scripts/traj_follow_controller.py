#!/usr/bin/env python
import numpy
import rospy
from std_msgs.msg import String
import mavros
from geometry_msgs.msg import PoseStamped,Vector3
from mavros_msgs.msg import State,AttitudeTarget,RCIn
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import tf.transformations
import sys, time
from nav_msgs.msg import Odometry

import timeit

from argparse import ArgumentParser

from pyquaternion import Quaternion

import roslaunch

class next_pos_sp:
	def __init__(self):
		self.vicon_data_cb_flag = False;
		self.trajectory_cb_flag = False;
		self.state_sub_cb_flag = False;

		#Rate init
		self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

		self.next_set_point_pub = rospy.Publisher("position_next", PoseStamped, queue_size=10)
		
		vicon_data_sub = rospy.Subscriber("intel_aero_quad/odom", Odometry, self.get_vicon_data_callback);
		trajectory_sub = rospy.Subscriber("points_traj", PoseArray, self.get_trajectory_callback);
		state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback);
		
		while not rospy.is_shutdown():
			if(self.rc_cb_flag == True and self.att_sp_cb_flag==True):


def traj_follow_controller():
	# pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('traj_follow_controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		traj_follow_controller()
	except rospy.ROSInterruptException:
		pass