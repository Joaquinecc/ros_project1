#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include <iostream>


// Global variables for goal parameters
double t_goal; // Time interval for publishing goals
double x_goal; // X-coordinate of the goal
double y_goal; // Y-coordinate of the goal

int main(int argc, char **argv)

{
	ros::init(argc, argv,"goaltalker");
	ros::NodeHandle node_obj; //= ros::NodeHandle("sdf");
	ros::Publisher number_publisher = node_obj.advertise<geometry_msgs::Point>("myGoals", 10);

	ros::NodeHandle nh_private("~");
	nh_private.param<double>("T_Goal", t_goal, 1.0);
	ros::Rate loop_rate(1/t_goal);
	while (ros::ok())
	{
		geometry_msgs::Point msg;

	  	nh_private.param<double>("X_Goal", x_goal, 1.0);
		nh_private.param<double>("Y_Goal", y_goal, 1.0);
		msg.x = x_goal;
		msg.y = y_goal;
        // Log the goal coordinates for debugging
        ROS_INFO("GOAL Point x, y = %f, %f", msg.x, msg.y);
		number_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
