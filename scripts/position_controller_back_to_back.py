#!/usr/bin/env python
import rospy
import mavros
import PID
import math
import sys
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import tf.transformations
import time

class test:
	""" params to add
	speed_controller_x_P
	speed_controller_x_I
	speed_controller_x_D
	speed_controller_y_P
	speed_controller_y_I
	speed_controller_y_D
	dy_sp
	dx_sp
	"""

	def __init__(self):
		self.vicon_cb_flag = False
		self.state_cb_flag = False
		self.update_pos_ball_sp_flag = False
		self.update_pos_car_sp_flag = False

		self.car_timer_flag = True
		self.ball_timer_flag = True

		self.time_threshold = rospy.get_param('/attitude_thrust_publisher/timer_threshold')


		self.vicon_yaw_sp = rospy.get_param('/attitude_thrust_publisher/vicon_yaw_sp')

		self.pos_sp_cb_flag = False

		self.update_pos_sp_flag = False

		# get the info for the PID and for the set point
		# PID Data
		self.dx_P = rospy.get_param('/attitude_thrust_publisher/position_controller_x_P')
		self.dx_I = 0
		self.dx_D = rospy.get_param('/attitude_thrust_publisher/position_controller_x_D')

		self.dy_P = rospy.get_param('/attitude_thrust_publisher/position_controller_y_P')
		self.dy_I = 0
		self.dy_D = rospy.get_param('/attitude_thrust_publisher/position_controller_y_D')


		self.vicon_dx_pid = PID.PID(self.dx_P, self.dx_I, self.dx_D)
		self.vicon_dy_pid = PID.PID(self.dy_P, self.dy_I, self.dy_D)

		self.x_sp = rospy.get_param('/attitude_thrust_publisher/x_sp')
		self.y_sp = rospy.get_param('/attitude_thrust_publisher/y_sp')

		# Rate of looping
		self.rate = rospy.Rate(200.0)
		# Information
		# Publisher for updating attitude for the speed
		# vicon_sum for current speed data
		self.attitude_target_speed_pub = rospy.Publisher("/px4_quad_controllers/rpy_setpoint", PoseStamped, queue_size=10)
		vicon_sub = rospy.Subscriber("/intel_aero_quad/odom", Odometry, self.vicon_subscriber_callback)
		rospy.Subscriber("/evdodge/evball/positionSetpoint", PoseStamped, self.ballPositionSetpoint_callback)
		rospy.Subscriber("/evdodge/evcar/positionSetpoint", PoseStamped, self.carPositionSetpoint_callback)
		while not rospy.is_shutdown():
			if (self.vicon_cb_flag): #If connected and data read in
				# start_time = time.time()
				# Update params if it was changed
				# SetPoint Data
				

				# PID Data
				self.dx_P = rospy.get_param('/attitude_thrust_publisher/position_controller_x_P')
				self.dx_I = 0
				self.dx_D = rospy.get_param('/attitude_thrust_publisher/position_controller_x_D')

				self.dy_P = rospy.get_param('/attitude_thrust_publisher/position_controller_y_P')
				self.dy_I = 0
				self.dy_D = rospy.get_param('/attitude_thrust_publisher/position_controller_y_D')
				# Update the PID with the speed data from vicon
				self.vicon_dx_pid.update(self.vicon_dx)
				self.vicon_dy_pid.update(self.vicon_dy)
				# print("--- %s seconds ---" % (time.time() - start_time))
				# Change the PID coefficients if they changed
				self.vicon_dx_pid.setKp(self.dx_P)
				self.vicon_dx_pid.setKi(self.dx_I)
				self.vicon_dx_pid.setKd(self.dx_D)
				self.vicon_dy_pid.setKp(self.dy_P)
				self.vicon_dy_pid.setKi(self.dy_I)
				self.vicon_dy_pid.setKd(self.dy_D)

				# rospy.loginfo(self.update_pos_sp_flag)
				if self.update_pos_car_sp_flag:
					if self.car_timer_flag:
						self.current_y = self.vicon_dy
						car_start_time = time.time()
						self.car_timer_flag = False
					car_timer_count = time.time() - car_start_time

					if car_timer_count<self.time_threshold:
						self.vicon_dy_pid.SetPoint = self.current_y + self.pos_update_y
					else:
						self.vicon_dy_pid.SetPoint = self.y_sp
				elif self.update_pos_ball_sp_flag:
					if self.ball_timer_flag:
						ball_start_time = time.time()
						self.ball_timer_flag = False
					ball_timer_count = time.time() - ball_start_time

					if ball_timer_count<self.time_threshold:
						self.vicon_dy_pid.SetPoint = self.y_sp + self.pos_update_y
					else:
						self.vicon_dy_pid.SetPoint = self.y_sp
				else:
					self.vicon_dy_pid.SetPoint = self.y_sp

				# print 'car'
				# print self.update_pos_car_sp_flag
				# print 'ball'
				# print self.update_pos_ball_sp_flag
				# rospy.loginfo('y sp'+str(self.vicon_dy_pid.SetPoint))
				self.vicon_dx_pid.SetPoint = self.x_sp

				vicon_y_output = self.vicon_dy_pid.output
				vicon_x_output = self.vicon_dx_pid.output
				target_attitude_speed = PoseStamped()
				target_attitude_speed.header.frame_id = "home"
				target_attitude_speed.header.stamp = rospy.Time.now()

				target_attitude_speed.pose.position.x = vicon_y_output
				target_attitude_speed.pose.position.y = vicon_x_output
				self.attitude_target_speed_pub.publish(target_attitude_speed)
			self.rate.sleep()

	def vicon_subscriber_callback(self,state):
		# Speed
		self.vicon_dx = state.pose.pose.position.x
		self.vicon_dy = state.pose.pose.position.y

		self.vicon_cb_flag = True

	def ballPositionSetpoint_callback(self,state):
		pos_update_temp_y = state.pose.position.y
		# rospy.loginfo(pos_update_temp_y)

		if pos_update_temp_y<1.5 and not self.update_pos_ball_sp_flag:
			self.pos_update_y = pos_update_temp_y
			self.update_pos_ball_sp_flag = True

	def carPositionSetpoint_callback(self,state):
		pos_update_temp_y = state.pose.position.y
		# rospy.loginfo(pos_update_temp_y)

		if pos_update_temp_y<1.5 and not self.update_pos_car_sp_flag:
			self.pos_update_y = pos_update_temp_y
			self.update_pos_car_sp_flag = True

def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
