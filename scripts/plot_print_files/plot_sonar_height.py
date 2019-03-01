#!/usr/bin/env python
import numpy
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,PositionTarget,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from px_comm.msg import OpticalFlow

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

		self.px4flow_cb_flag = False
		startup_start = timeit.default_timer()
		print('start')

		#Rate init
		self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

		self.current_state = State()
		#Current state sub
		vicon_sub = rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, self.vicon_sub_callback)
		self.px4flow_flow_pub = rospy.Publisher("/px4_quad_controllers/px4flow_plot", PoseStamped, queue_size=10)
		self.px4flow_vel_pub = rospy.Publisher("/px4_quad_controllers/px4flow_vel_plot", PoseStamped, queue_size=10)

		while not rospy.is_shutdown():
			if(self.px4flow_cb_flag == True):

				px4flow_flow = PoseStamped()
				px4flow_flow.header.frame_id = "home"
				px4flow_flow.header.stamp = rospy.Time.now()
				px4flow_flow.pose.position.x = self.flow_x
				px4flow_flow.pose.position.y = self.flow_y
				px4flow_flow.pose.position.z = self.height_feedback

				px4flow_vel = PoseStamped()
				px4flow_vel.header.frame_id = "home"
				px4flow_vel.header.stamp = rospy.Time.now()
				px4flow_vel.pose.position.x = self.vel_x
				px4flow_vel.pose.position.y = self.vel_y

				self.px4flow_flow_pub.publish(px4flow_flow)
				self.px4flow_vel_pub.publish(px4flow_vel)
			self.rate.sleep()

	def vicon_sub_callback(self,state):
		self.height_feedback = state.ground_distance
		self.flow_x = state.flow_x
		self.flow_y = state.flow_y
		self.vel_x = state.velocity_x
		self.vel_y = state.velocity_y
		self.px4flow_cb_flag = True

def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
