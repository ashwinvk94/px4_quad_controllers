#!/usr/bin/env python
import numpy
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,PositionTarget,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry

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
		self.imu_cb_flag = False
		self.vicon_cb_flag = False
		self.traj_cb_flag = False

		self.local_pose_subscriber_prev=0
		startup_start = timeit.default_timer()
		print('start')

		#Rate init
		self.rate = rospy.Rate(1.0)

		self.current_state = State()


		#Init last_request
		self.last_request = rospy.get_rostime()

		#Quadcopter imu subscriber init
		vicon_attitude_sub = rospy.Subscriber("/intel_aero_quad/odom", Odometry, self.vicon_attitude_sub_callback)
		
		#Quadcopter imu subscriber init
		attitude_target_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.attitude_target_sub_callback)
		traj_yaw_sub = rospy.Subscriber("/px4_quad_controllers/traj_yaw", PoseStamped, self.traj_yaw_sub_sub_callback)

		self.yaw_target_pub = rospy.Publisher("/px4_quad_controllers/yaw_setpoint", PoseStamped, queue_size=10)


		while not rospy.is_shutdown():
			if(self.vicon_cb_flag == True and self.imu_cb_flag == True and self.traj_cb_flag == True):

				self.vicon_yaw_sp = rospy.get_param('/attitude_thrust_publisher/vicon_yaw_sp')

				self.yaw_sp = self.current_yaw_vicon - (self.vicon_yaw_sp+self.traj_yaw_sp)

				target_yaw = PoseStamped()
				target_yaw.header.frame_id = "home"
				target_yaw.header.stamp = rospy.Time.now()
				target_yaw.pose.position.x = self.current_yaw_imu - self.yaw_sp

				self.yaw_target_pub.publish(target_yaw)

				#print('VICON setpoint yaw = '+ str(self.vicon_yaw_sp))
				#print('VICON current yaw = '+ str(self.current_yaw_vicon))

				#print('quad CURRENT yaw = '+ str(self.current_yaw_imu))

				#print('final setpoint yaw = '+ str(target_yaw.pose.position.x))

				#print('\n')
			self.rate.sleep()

		
	def vicon_attitude_sub_callback(self,state):
		orientation = (state.pose.pose.orientation)
		#self.ground_wrt_body_quat = Quaternion(orientation.w,orientation.x,orientation.y,orientation.z)
		#quat = state.orientation
		#https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
		euler = tf.transformations.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
		self.current_yaw_vicon = euler[0]
		self.vicon_cb_flag = True


	def traj_yaw_sub_sub_callback(self,state):
		self.traj_yaw_sp = state.pose.position.x
		self.traj_cb_flag = True

	def attitude_target_sub_callback(self,state):
		orientation = (state.orientation)
		euler = tf.transformations.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
		self.current_yaw_imu = euler[0]
		self.imu_cb_flag = True


def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
