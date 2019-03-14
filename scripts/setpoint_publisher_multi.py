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
		self.object_cutoff_line_x = rospy.get_param('/attitude_thrust_publisher/object_cutoff_line_x')

		self.target_position_pub = rospy.Publisher("/evdodge/positionSetpoint", PoseStamped, queue_size=10)
		rospy.Subscriber("/Ball_ev/odom",Odometry,self.ball_vicon_subscriber_callback)
		rospy.Subscriber("/Car_ev/odom",Odometry,self.car_vicon_subscriber_callback)

		while not rospy.is_shutdown():
			if (self.car_cb_flag and self.ball_cb_flag):
				self.move_position = [0,self.ball_dy+self.car_dy,self.z_sp_update]
			
			
			target_attitude_speed = PoseStamped()
			target_attitude_speed.header.frame_id = "home"
			target_attitude_speed.header.stamp = rospy.Time.now()


			target_attitude_speed.pose.position.x = self.move_position[0]
			target_attitude_speed.pose.position.y = self.move_position[1]*0.5
			target_attitude_speed.pose.position.z = self.move_position[2]

			self.target_position_pub.publish(target_attitude_speed)
			self.rate.sleep()
	def ball_vicon_subscriber_callback(self, state):
		self.ball_x = state.pose.pose.position.x

		self.ball_dx = state.twist.twist.linear.x
		ball_dy = state.twist.twist.linear.y
		self.ball_dz = state.twist.twist.linear.z

		# print('update')
		if self.ball_x>self.object_cutoff_line_x and self.ball_cb_flag==False:
			norm = math.sqrt(self.ball_dx**2+ball_dy**2+self.ball_dz**2)
			self.ball_dy = ball_dy/norm
			self.ball_cb_flag = True
		# print 'ball'
		# print self.ball_x
		# print self.ball_cb_flag

	def car_vicon_subscriber_callback(self, state):
		self.car_x = state.pose.pose.position.x

		self.car_dx = state.twist.twist.linear.x
		car_dy = state.twist.twist.linear.y
		self.car_dz = state.twist.twist.linear.z

		# print('update')
		if self.car_x>self.object_cutoff_line_x and self.car_cb_flag==False:
			norm = math.sqrt(self.car_dx**2+car_dy**2+self.car_dz**2)
			self.car_dy = car_dy/norm 
			self.car_cb_flag = True
		# print 'car'
		# print self.car_x
		# print self.car_cb_flag

def main(args):
	rospy.init_node('positionSetpointPub', anonymous=True)
	ic = positionSetpointPub()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__ == '__main__':
	main(sys.argv)
