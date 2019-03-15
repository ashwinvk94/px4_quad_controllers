#!/usr/bin/env python2
import rospy
import sys
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class positionSetpointPub:

	def __init__(self):
		self.quad_vicon_cb_flag = False
		self.state_cb_flag = False
		self.pos_sp_cb_flag = False
		self.mag = 0.5
		self.distance_thershold = 2
		self.flag = False
		# Rate of looping
		self.rate = rospy.Rate(200.0)
		self.move_position = [0,0,0]
		# Information
		self.ball_cutoff_line_x = rospy.get_param('/attitude_thrust_publisher/ball_cutoff_line_x')

		# Publisher for updating attitude for the speed
		self.target_position_pub = rospy.Publisher("/evdodge/positionSetpoint", PoseStamped, queue_size=10)
		rospy.Subscriber("/Ball_ev/odom",Odometry,self.ball_vicon_subscriber_callback)
		# rospy.Subscriber("/bebop_ev/odom",Odometry,self.ball_vicon_subscriber_callback)
		rospy.Subscriber("/intel_aero_quad/odom",Odometry,self.quad_vicon_subscriber_callback)

	def ball_vicon_subscriber_callback(self, state):
		self.ball_x = state.pose.pose.position.x

		self.ball_dx = state.twist.twist.linear.x
		self.ball_dy = state.twist.twist.linear.y
		self.ball_dz = state.twist.twist.linear.z
		norm = math.sqrt(self.ball_dx**2+self.ball_dy**2+self.ball_dz**2)
		self.ball_dy = self.ball_dy/norm 
		self.ball_dz = self.ball_dz/norm

		# print('update')
		if self.ball_x>self.ball_cutoff_line_x and self.flag==False:
			if self.ball_dy>0:
				self.perpend_vector = [0,-self.ball_dz,self.ball_dy]
			else:
				self.perpend_vector = [0,self.ball_dz,-self.ball_dy]
			self.move_position = self.perpend_vector
			self.flag = True
		# print(self.move_position)

		target_attitude_speed = PoseStamped()
		target_attitude_speed.header.frame_id = "home"
		target_attitude_speed.header.stamp = rospy.Time.now()

		target_attitude_speed.pose.position.x = self.move_position[0]
		target_attitude_speed.pose.position.y = self.move_position[1]*0.5
		target_attitude_speed.pose.position.z = self.move_position[2]*0.5

		self.target_position_pub.publish(target_attitude_speed)

		self.vicon_cb_flag = True

	def quad_vicon_subscriber_callback(self, state):
		# Speed
		self.quad_x = state.pose.pose.position.x
		self.quad_y = state.pose.pose.position.y
		self.quad_z = state.pose.pose.position.z 

		self.quad_vicon_cb_flag = True

 #    def DotProduct(x, y):
 #    	return sum([x[i] * y[i] for i in range(len(x))])

	# def norm(self,x):
	#     return sqrt(DotProduct(x, x))

	# def normalize(self,x):
	#     return [x[i] / norm(x) for i in range(len(x))]

	# def projection(self,x, n):
	#     d = DotProduct(x, n) / norm(n)
	#     proj = [d * normalize(n)[i] for i in range(len(n))]
	#     return [x[i] - proj[i] for i in range(len(x))]


def main(args):
	rospy.init_node('positionSetpointPub', anonymous=True)
	ic = positionSetpointPub()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__ == '__main__':
	main(sys.argv)
