import rospy
import mavros
import PID
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,
from geometry_msgs.msg import Twist
import tf.transformations

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


		self.vicon_yaw_sp = rospy.get_param('/attitude_thrust_publisher/vicon_yaw_sp')

		self.pos_sp_cb_flag = False

        # get the info for the PID and for the set point
		updateParamData()

		self.vicon_dx_pid = PID.PID(self.dx_P, self.dx_I, self.dx_D)
		self.vicon_dy_pid = PID.PID(self.dy_P, self.dy_I, self.dy_D)

        # Rate of looping
        self.rate = rospy.Rate(20.0)
        # Information
        # Publisher for updating attitude for the speed
        # vicon_sum for current speed data
		self.attitude_target_speed_pub = rospy.Publisher("/px4_quad_controllers/rpy_setpoint", PoseStamped, queue_size=10)
        vicon_sub = rospy.Subscriber("/intel_aero_quad/odom", Odometry, self.vicon_subscriber_callback)
        while not rospy.is_shutdown():
            if (self.vicon_cb_flag and self.state_cb_flag): #If connected and data read in
                # Update params if it was changed
                updateParamData()
                updatePID()
                vicon_y_output = self.vicon_y_pid.output
				vicon_x_output = -self.vicon_x_pid.output
				target_attitude_speed = PoseStamped()
				target_attitude_speed.header.frame_id = "home"
				target_attitude_speed.header.stamp = rospy.Time.now()

				target_attitude_speed.pose.position.x = vicon_y_output
				target_attitude_speed.pose.position.y = vicon_x_output
				self.attitude_target_speed_pub.publish(target_attitude_speed)

            self.rate.sleep()

    # Current state subscriber
    # To ensure looping while active
	def state_subscriber_callback(self,state):
		self.current_state = state.mode
		self.state_cb_flag = state.connected # ask (to ensure that it is connected)

    def updateParamData():
        # SetPoint Data
        self.dx_sp = rospy.get_param('/attitude_thrust_publisher/dx_sp')
		self.dy_sp = rospy.get_param('/attitude_thrust_publisher/dy_sp')

        # PID Data
        self.dx_P = rospy.get_param('/attitude_thrust_publisher/speed_controller_x_P')
        self.dx_I = rospy.get_param('/attitude_thrust_publisher/speed_controller_x_I')
		self.dx_D = rospy.get_param('/attitude_thrust_publisher/speed_controller_x_D')

    	self.dy_P = rospy.get_param('/attitude_thrust_publisher/speed_controller_y_P')
        self.dy_I = rospy.get_param('/attitude_thrust_publisher/speed_controller_y_I')
		self.dy_D = rospy.get_param('/attitude_thrust_publisher/speed_controller_y_D')

    def updatePID():
        # Update the PID with the speed data from vicon
        self.vicon_dx_pid.update(self.vicon_dx)
        self.vicon_dy_pid.update(self.vicon_dy)

        # Change the PID coefficients if they changed
        self.vicon_dx_pid.setKp(self.dx_P)
        self.vicon_dx_pid.setKi(self.dx_I)
        self.vicon_dx_pid.setKd(self.dx_D)
        self.vicon_dy_pid.setKp(self.dy_P)
        self.vicon_dy_pid.setKi(self.dy_I)
        self.vicon_dy_pid.setKd(self.dy_D)

        self.vicon_dy_pid.SetPoint = self.dy_sp
        self.vicon_dx_pid.SetPoint = self.dx_sp


    def vicon_subscriber_callback(self,state):
        # Speed
        self.vicon_dx = state.twist.twist.linear.x
    	self.vicon_dy = state.twist.twist.linear.y

        # Get the yaw because this will affect the angle adjustment
        orientation = (state.pose.pose.orientation)
		#https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
		euler = tf.transformations.euler_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z])
		self.current_yaw_vicon = euler[0]
		self.yaw_change = self.yaw_change = self.vicon_yaw_sp - self.current_yaw_vicon
		self.vicon_cb_flag = True

def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
