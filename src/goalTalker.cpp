/*
 * Copyright (C) 2015, Lentin Joseph and Qbotics Labs Inc.
 * Email id : qboticslabs@gmail.com
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
* This code will publish a integers from 0 to n with a delay of 100ms
 */

//roscpp Core header
#include "ros/ros.h"
//Header of Int32 standard message
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include <iostream>


double t_goal;
double x_goal;
double y_goal;

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
		ROS_INFO("GOAL Point x,y = %f %f",msg.x, msg.y);
		number_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
