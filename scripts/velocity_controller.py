#!/usr/bin/env python
import numpy
import rospy
import math
import mavros
from geometry_msgs.msg import PoseStamped,Vector3,TwistStamped
from mavros_msgs.msg import State,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
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
		self.state_cb_flag = False

		self.vel_sp_cb_flag = False
		# setting params
		self.vicon_yaw_sp = rospy.get_param('/attitude_thrust_publisher/vicon_yaw_sp')
		self.P = rospy.get_param('/attitude_thrust_publisher/velocity_controller_P')
		self.I = 0
		self.D = rospy.get_param('/attitude_thrust_publisher/velocity_controller_D')
		#calculating PID
		self.vicon_y_pid = PID.PID(self.P, self.I, self.D)
		self.vicon_x_pid = PID.PID(self.P, self.I, self.D)

		#X axis of the vicon system should alwasy be aligned with the front of the quad

		#Rate init
		#DECIDE ON PUBLISHING RATE
		self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

		#PUBLISHERS
		self.attitude_target_pub = rospy.Publisher("/px4_quad_controllers/rpy_setpoint", PoseStamped, queue_size=10)

		#SUBSCRIBERS
		vicon_sub = rospy.Subscriber("/intel_aero_quad/odom", Odometry, self.vicon_subscriber_callback)
		state_sub = rospy.Subscriber("/mavros/state", State, self.state_subscriber_callback)
		vel_sp_sub = rospy.Subscriber("/px4_quad_controllers/vel_setpoint", TwistStamped, self.vel_sp_subscriber_callback)


		while not rospy.is_shutdown():
			self.P = rospy.get_param('/attitude_thrust_publisher/velocity_controller_P')
			self.D = rospy.get_param('/attitude_thrust_publisher/velocity_controller_D')
			self.vicon_y_pid.setKp(self.P)
			self.vicon_y_pid.setKd(self.D)
			self.vicon_x_pid.setKp(self.P)
			self.vicon_x_pid.setKd(self.D)
		
			if(self.vicon_cb_flag==True and self.state_cb_flag==True):
				
				#Update setpoint

				if(self.vel_sp_cb_flag==False):
					self.vel_y_sp = rospy.get_param('/attitude_thrust_publisher/vel_y_sp')
					self.vel_x_sp = rospy.get_param('/attitude_thrust_publisher/vel_x_sp')
				else:
					self.vel_y_sp = self.vel_sp_y_from_pos_ctrl
					self.vel_x_sp = self.vel_sp_x_from_pos_ctrl
				
				self.vicon_y_pid.SetPoint = self.vel_y_sp
				self.vicon_x_pid.SetPoint = self.vel_x_sp

				self.vicon_y_pid.update(self.vicon_y_pos)
				self.vicon_x_pid.update(self.vicon_x_pos)

				
				vicon_y_output = self.vicon_y_pid.output
				vicon_x_output = -self.vicon_x_pid.output
				target_attitude = PoseStamped()
				target_attitude.header.frame_id = "home"
				target_attitude.header.stamp = rospy.Time.now()
				target_attitude.pose.position.x = -vicon_y_output * math.cos(self.yaw_change) - vicon_x_output * math.sin(self.yaw_change) #roll -
				target_attitude.pose.position.y = -vicon_y_output * math.sin(self.yaw_change) +  vicon_x_output * math.cos(self.yaw_change) #pitch

				self.attitude_target_pub.publish(target_attitude)

			self.rate.sleep()

	def vicon_subscriber_callback(self,state):
		self.vicon_x_pos = state.twist.twist.linear.x
		self.vicon_y_pos = state.twist.twist.linear.y
		orientation = (state.pose.pose.orientation)
		#self.ground_wrt_body_quat = Quaternion(orientation.w,orientation.x,orientation.y,orientation.z)
		#quat = state.orientation
		#https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
		euler = tf.transformations.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
		self.current_yaw_vicon = euler[0]
		self.yaw_change = self.yaw_change = self.vicon_yaw_sp - self.current_yaw_vicon
		self.vicon_cb_flag = True

	def vel_sp_subscriber_callback(self,state):
		self.vel_sp_x_from_pos_ctrl = state.twist.linear.x
		self.vel_sp_y_from_pos_ctrl = state.twist.linear.y
		self.vel_sp_cb_flag = True

	#Current state subscriber
	def state_subscriber_callback(self,state):
		self.current_state = state.mode
		self.state_cb_flag = True

		self.rate.sleep()
def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
