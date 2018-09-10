#include "ros/ros.h"
// #include <mav_trajectory_generation/polynomial_optimization_linear.h>
// #include <mav_trajectory_generation/trajectory.h>
// #include <mav_trajectory_generation/trajectory_sampling.h>
// #include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
using namespace std;

//****Subscribers
ros::Subscriber vicon_data_sub;
ros::Subscriber trajectory_sub;
ros::Subscriber state_sub;
//****Publishers
ros::Publisher next_set_point_pub;
// ros::Publisher setpoint_pub;
// ros::Publisher mocap_pub;
// ros::Publisher pos_sp_pub;

//constants
visualization_msgs::MarkerArray markers;
double distance = 1.6; 						// Distance by which to seperate additional markers. Set 0.0 to disable.
std::string frame_id = "/vicon";
geometry_msgs::PoseArray trajectory_pts;
// nav_msgs::Odometry odom_feedback;
geometry_msgs::Point current_position;
string mode_;
bool is_odom_detected = false;
int trajectory_pts_size = 0;
int  prev_index = 0,yaw_reset = 0;
double localization_error = 0.10;						//m,
geometry_msgs::PoseStamped next_set_point;
geometry_msgs::PoseStamped prev_set_point;
bool trajectory_cb_flag = false;
bool vicon_data_cb_flag = false;
bool state_cb_flag = false;

 // A Subscriber function to get trajectory points
void get_trajectory_callback(const geometry_msgs::PoseArray::ConstPtr& pts)
{
	// cout<<"inside traj cb"<<endl;
	// trajectory_pts.poses.
	trajectory_pts_size = trajectory_pts.poses.size();
	// cout<<"trajectory array size = "<<trajectory_pts_size<<endl;
	trajectory_pts = *pts;
	trajectory_cb_flag = true;
}

// A Subscriber function to get odometry data form vicon
void get_vicon_data_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
	// cout<<"inside vicon cb"<<endl;
	current_position.x = odom_msg->pose.pose.position.x;
	current_position.y = odom_msg->pose.pose.position.y;
	current_position.z = odom_msg->pose.pose.position.z;
	is_odom_detected = true;
	vicon_data_cb_flag = true;
}

// A Subscriber function to get state; onboard or offboard
void state_callback(const mavros_msgs::State::ConstPtr &msg)
{
	// cout<<"inside mavros cb"<<endl;
	mode_ = msg->mode;
	state_cb_flag = true;
}

double get_distance(double x, double y, double z, double a, double b, double c)
{
	double distance = sqrt( pow((x-a),2) + pow((y-b),2) + pow((z-c),2));
	return distance;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "traj_follow_controller");
	ros::NodeHandle n;
	cout<<" d1"<<endl;
	if (argc < 2)
	{
		cout << "Enter the following arguments ..\n"
            "1) localization_error (default = 0.10 meter)\n"
            "\n";
        return -1;
	}

  localization_error = atof(argv[1]);

// cout<<" d2"<<endl;
	vicon_data_sub = n.subscribe<nav_msgs::Odometry>("intel_aero_quad/odom",50, get_vicon_data_callback);
	trajectory_sub = n.subscribe<geometry_msgs::PoseArray>("points_traj", 10, get_trajectory_callback);
	state_sub = n.subscribe<mavros_msgs::State>("/mavros/state",5, state_callback);
	// cout<<" d3"<<endl;
	next_set_point_pub = n.advertise<geometry_msgs::PoseStamped>("/px4_quad_controllers/pos_sp",10);
	// cout<<" d4"<<endl;

	ros::Rate loop_rate(30);
	bool is_initial_condition = true;
cout<<" d5"<<endl;
	int count = 0;
	while (ros::ok())
	{	
		ros::spinOnce();
		// cout<<" d6"<<endl;
		//Step 1: Localization-where are we? 
		//Current position? .....Vicon data = odom_feedback.
		if(trajectory_cb_flag == true)
		{
			if (is_initial_condition)
			{
				// cout<<" x pos = "<<trajectory_pts.poses[5].position.x<<endl;
				next_set_point.pose = trajectory_pts.poses.at(0);		//initialize to start set point
				prev_set_point.pose = trajectory_pts.poses.at(0);
				// next_set_point.header.stamp = ros::Time::now();
				prev_index = 0;
				is_initial_condition = false;
			}

	// cout<<" d7"<<endl;
			// mode_ = "OFFBOARD";
			if(mode_ == "OFFBOARD")
			{
				next_set_point.pose = trajectory_pts.poses.at(0);
				prev_set_point.pose = trajectory_pts.poses.at(0);
				next_set_point.header.stamp = ros::Time::now();
				next_set_point_pub.publish(next_set_point);
				cout<<"pub in OFFBOARD mode"<<endl;
			}
			else
			{
				// int number_of_closest_waypoints = 0;
				for(int index=0; index < trajectory_pts_size; index++)
		        {
		            float x_sp = trajectory_pts.poses.at(index).position.x;
		            float y_sp = trajectory_pts.poses.at(index).position.y;
		            float z_sp = trajectory_pts.poses.at(index).position.z;

		            double distance_between_points = get_distance(x_sp, y_sp, z_sp, current_position.x, current_position.y, current_position.z);
		            cout<<"distance_between_points = "<<distance_between_points<<endl;

		            if(distance_between_points < localization_error && index > prev_index)
		            {
		                prev_index = index;
		                cout<<"index = "<<index;
		                next_set_point.pose = trajectory_pts.poses.at(index);
		                next_set_point.header.stamp = ros::Time::now();
		                prev_set_point= next_set_point;
		                next_set_point_pub.publish(next_set_point);
		                ROS_INFO("Found next set point");
		                // next_set_point.position.x = x_sp;
		                // next_set_point.position.y = y_sp;
		            }
		            else
		            {
						next_set_point_pub.publish(prev_set_point);
		            }
		        }

		        cout<<" d8"<<endl;
		     //    if(number_of_closest_waypoints = 1)
		    	// {
		    		//case 1 prev_point is start
		    		//case 2 prev_point is different than current
		    		//case 3 prev_point is same as current but isn't start
		    	// }
		    	// else
		    	// 	{
			    // 		//solve this edge case
		    	// 	}
	    	}
	    }
			

		loop_rate.sleep();
	}


	return 0;
}
