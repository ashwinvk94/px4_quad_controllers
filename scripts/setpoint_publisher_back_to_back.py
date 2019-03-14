#!/usr/bin/env python2
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class positionSetpointPub:

	def __init__(self):
		self.quad_vicon_cb_flag = False
		self.ball_cb_flag = False
		self.car_cb_flag = False
		self.state_cb_flag = False
		self.pos_sp_cb_flag = False
		self.mag = 1
		self.distance_thershold = 2
		self.flag = False
		# Rate of looping
		self.rate = rospy.Rate(200.0)
		self.move_position = [0,0,0]
		# Information
		# Publisher for updating attitude for the speed
		self.z_sp_update = rospy.get_param('/attitude_thrust_publisher/z_sp_update')
		self.ball_cutoff_line_x = rospy.get_param('/attitude_thrust_publisher/ball_cutoff_line_x')
		self.car_cutoff_line_x = rospy.get_param('/attitude_thrust_publisher/car_cutoff_line_x')

		self.target_position_ball_pub = rospy.Publisher("/evdodge/evball/positionSetpoint", PoseStamped, queue_size=10)
		self.target_position_car_pub = rospy.Publisher("/evdodge/evcar/positionSetpoint", PoseStamped, queue_size=10)
		rospy.Subscriber("/Ball_ev/odom",Odometry,self.ball_vicon_subscriber_callback)
		rospy.Subscriber("/Car_ev/odom",Odometry,self.car_vicon_subscriber_callback)

		while not rospy.is_shutdown():
			if (self.ball_cb_flag):
				# self.move_position = [0,self.ball_dy,self.z_sp_update]
				target_attitude_speed = PoseStamped()
				target_attitude_speed.header.frame_id = "home"
				target_attitude_speed.header.stamp = rospy.Time.now()


				target_attitude_speed.pose.position.x = self.move_position[0]
				target_attitude_speed.pose.position.y = self.move_position[1]*0.3
				target_attitude_speed.pose.position.z = self.move_position[2]
				self.target_position_ball_pub.publish(target_attitude_speed)
			if (self.car_cb_flag):
				# self.move_position = [0,self.car_dy,self.z_sp_update]
				target_attitude_speed = PoseStamped()
				target_attitude_speed.header.frame_id = "home"
				target_attitude_speed.header.stamp = rospy.Time.now()


				target_attitude_speed.pose.position.x = self.move_position[0]
				target_attitude_speed.pose.position.y = self.move_position[1]*0.3
				target_attitude_speed.pose.position.z = self.move_position[2]
				self.target_position_car_pub.publish(target_attitude_speed)

			self.rate.sleep()

	def ball_vicon_subscriber_callback(self, state):
		self.ball_x = state.pose.pose.position.x

		self.ball_dx = state.twist.twist.linear.x
		ball_dy = state.twist.twist.linear.y
		ball_dz = state.twist.twist.linear.z

		# print('update')
		if self.ball_x>self.ball_cutoff_line_x and self.ball_cb_flag==False:
			norm = math.sqrt(self.ball_dx**2+ball_dy**2+ball_dz**2)
			self.ball_dy = ball_dy/norm
			self.ball_dz = ball_dz/norm
			if self.ball_dy>0:
				self.perpend_vector = [0,-self.ball_dz,self.ball_dy]
			else:
				self.perpend_vector = [0,self.ball_dz,self.z_sp_update]
			self.move_position = self.mag*self.perpend_vector
			self.ball_cb_flag = True

	def car_vicon_subscriber_callback(self, state):
		self.car_x = state.pose.pose.position.x

		self.car_dx = state.twist.twist.linear.x
		car_dy = state.twist.twist.linear.y
		car_dz = state.twist.twist.linear.z

		# print('update')
		if self.car_x>self.car_cutoff_line_x and self.car_cb_flag==False:
			norm = math.sqrt(self.car_dx**2+car_dy**2+car_dz**2)
			self.car_dy = car_dy/norm
			self.car_dz = car_dz/norm

			if self.car_dy>0:
				self.perpend_vector = [0,-self.car_dz,self.z_sp_update]
			else:
				self.perpend_vector = [0,self.car_dz,self.z_sp_update]
			self.move_position = self.mag*self.perpend_vector
			self.car_cb_flag = True

def main(args):
	rospy.init_node('positionSetpointPub', anonymous=True)
	ic = positionSetpointPub()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__ == '__main__':
	main(sys.argv)
