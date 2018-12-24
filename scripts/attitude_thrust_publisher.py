#!/usr/bin/env python2
# import numpy
import rospy
# import mavros
from geometry_msgs.msg import PoseStamped  # , Vector3
from mavros_msgs.msg import State,AttitudeTarget, RCIn  # , AttitudeTarget
# from mavros_msgs.srv import CommandBool

# from std_msgs.msg import Float32, Bool
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf.transformations
import sys
# import time
import PID

# import timeit

# from argparse import ArgumentParser

# from pyquaternion import Quaternion

# import roslaunch


class test:
	def __init__(self):
		

		# Rate init
		# DECIDE ON PUBLISHING RATE
		self.thrust_sp_cb_flag = False
		self.rc_cb_flag = False
		self.rate = rospy.Rate(500.0)  # MUST be more then 2Hz

		# ADD SUBSCRIBER FOR VICON DATA
		rc_sub = rospy.Subscriber("/mavros/rc/in",RCIn,self.rc_callback)
		self.attitude_thrust_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
		thrust_target_sub = rospy.Subscriber("/px4_quad_controllers/thrust_setpoint",PoseStamped,self.thrust_setpoint_sub_callback)
		self.yaw_sp = 0
		while not rospy.is_shutdown():
			#print(self.rc_cb_flag)
			if(self.rc_cb_flag==True and self.thrust_sp_cb_flag==True):

				# USE THE NEXT 8 LINES ONLY FOR INITIAL TESTING
				#rc_roll = -0.1
				#rc_pitch = -0.2
				#self.yaw_sp = 0.1
				#thrust_sp = 0
				

				# Manual control
				self.roll_sp = self.rc_roll
				self.pitch_sp = self.rc_pitch
				#self.thrust_sp = self.rc_thrust

				#Autonmous control
				self.thrust_sp = self.thrust_sp

				if self.rc_thrust>0.05:
					self.yaw_sp = self.yaw_sp+self.rc_yawrate
				
				#print 'att_r'+str(self.roll_sp)
				#print 'att_p'+str(self.pitch_sp)
				#print 'att_y'+str(self.yaw_sp)
				#print 'thrust_sp'+str(self.thrust_sp)
				#print('\n')
				att_quat_w, att_quat_x, att_quat_y, att_quat_z = tf.transformations.quaternion_from_euler(self.yaw_sp, self.pitch_sp,self.roll_sp, axes='sxyz')

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

	def rc_callback(self, msg):
		self.rc_roll = (float(msg.channels[0]) - 1500) / 1000
		self.rc_pitch = -(float(msg.channels[1]) - 1500) / 1000
		self.rc_thrust = (float(msg.channels[2]) - 1000) / 1000
		self.rc_yawrate = (float(msg.channels[3]) - 1500) / 50000
		self.rc_cb_flag = True

	def thrust_setpoint_sub_callback(self, state):
        self.thrust_sp = state.pose.position.x
        self.thrust_sp_cb_flag = True


def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic = test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


if __name__ == '__main__':
	main(sys.argv)
