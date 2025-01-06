/**
 *  Created on: January 2025
 *  Author: Joaquin Caballero
 *  Example of odometry, laser and command vel topics
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "assert.h"
#include "math.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"


#include <string>
#include <sstream>
#include <iostream>


using namespace std;
using std::ostringstream;


// Variable definitions with type and default values
int ID_ROBOT = 1;
int ALGOR = 1;
double CRIT_DIST = 1.0;
double D_OBJ = 0.5;
double V_MAX_DES = 1.0;
double V_MAX_ROT = 1.0;
double K_ROT_MIN = 0.1;
double K_ROT_MAX = 0.25;
double ORI_ERROR = 0.4;
double T_AVOID_OBS = 2.0;
double DIST_LEADER = 2.0;
int ROBOT_ROL = 0;
int ID_LEADER = 0;
double W_1 = 1.5;
double W_2 = 3.0;
int T_WAIT = 150;


ros::Publisher cmd_vel_pub; // publisher for movement commands
ros::Time start;

double current_Xr;
double current_Yr;
double current_z;
double current_w;
double current_orientationr;


geometry_msgs::Point globalGoal;
//States Flag
bool isTheregoal = false;
bool isThereobstacle = false;
bool avoidObstacle = false;
bool DEBUG=true;

double calculateOrientationDifference(double goal_x, double goal_y) {
/**
 * @brief Calculate the difference between the current orientation and the goal orientation.
 * 
 * Given the goal point (goal.x, goal.y) and the current robot position (current_Xr, current_Yr),
 * this function calculates the orientation difference in radians.
 * 
 * @param goal_x X-coordinate of the goal point.
 * @param goal_y Y-coordinate of the goal point.
 * @return double The orientation difference in radians.
 */
    // Calculate the desired angle to the goal point
    double desired_orientation = atan2(goal_y - current_Yr, goal_x - current_Xr);

    // Calculate the difference in orientation
    double orientation_difference = desired_orientation - current_orientationr;

    // Normalize the orientation difference to the range [-π, π]
    while (orientation_difference > M_PI) {
        orientation_difference -= 2.0 * M_PI;
    }
    while (orientation_difference < -M_PI) {
        orientation_difference += 2.0 * M_PI;
    }

    return orientation_difference;
}

/**
  Read the the Laser data. If there is an obstacle close r the robot, stop and turn
  otherwise move ahead. 
*/

geometry_msgs::Twist algo1(const sensor_msgs::LaserScan& most_intense) {
    //Set all to 0
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // Detect obstacles
    bool obstacle_detected = false; // Local flag to detect obstacles
    int length = most_intense.ranges.size(); 		
    for(int i=0; i<length; i++){
        if(most_intense.ranges[i] < CRIT_DIST) {
            obstacle_detected= true;
            cmd_vel.angular.z = 1.0;
            break;}
    }
    if (obstacle_detected) {
        if(DEBUG && !isThereobstacle){// Only the first ime
            ROS_INFO("obstacle_detected");
        }
        // Set obstacle flags and rotate in place
        isThereobstacle = true;
        cmd_vel.angular.z = 1.0; // Rotate to avoid obstacle
    }else{// No obstacle detected
        if (isThereobstacle) {
            if(DEBUG){
    			ROS_INFO("Avoid Obstacle State");
            }
            // Transition from obstacle state to avoid obstacle
            avoidObstacle = true;
            isThereobstacle = false; // Reset obstacle flag
        }
        if (avoidObstacle) {
            // Move forward after avoiding obstacle
            cmd_vel.linear.x = V_MAX_DES;
        } else {
            // Normal operation: move toward goal
            double diff_angle =  calculateOrientationDifference(globalGoal.x, globalGoal.y);
            // Proportional Controller for rotation
            double rot_vel = 0.0;
            if (fabs(diff_angle) < ORI_ERROR) {
                rot_vel = K_ROT_MIN * V_MAX_ROT * diff_angle;
                cmd_vel.linear.x = V_MAX_DES;
            } else {
                rot_vel = K_ROT_MAX * V_MAX_ROT * diff_angle;
            }
            cmd_vel.angular.z = rot_vel;
        }
    }
    return cmd_vel;
}


geometry_msgs::Twist algo2(const sensor_msgs::LaserScan& most_intense) {
		//Set all to 0
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;

	// Calculate Vobj (Go to Goal Behavior)
	double VobjX = globalGoal.x - current_Xr;
	double VobjY = globalGoal.y - current_Yr;
	double normVobj = std::sqrt(VobjX * VobjX + VobjY * VobjY);
	VobjX /= normVobj; // Normalize to unit vector
    VobjY /= normVobj;

    // Calculate Vobs (Avoid Obstacles Behavior)
    double VobsX = 0.0, VobsY = 0.0;
    int length = most_intense.ranges.size();

    for (int i = 0; i < length; ++i) {
        double angle = most_intense.angle_min + i * most_intense.angle_increment;
        double di = most_intense.ranges[i];

        // Skip invalid readings
        if (di < 0.01 || di > most_intense.range_max) {
            continue;
        }

        // Calculate repulsion magnitude
        double magnitude = (di < CRIT_DIST) ? (CRIT_DIST-di) / CRIT_DIST : 0.0;

        // Add to Vobs
        VobsX += magnitude * std::cos(angle);
        VobsY += magnitude * std::sin(angle);
    }

	// Calculate resulting vector Vf
    double VfX = W_1 * VobjX - W_2 * VobsX;
    double VfY = W_1 * VobjY - W_2 * VobsY;

    // Calculate desired orientation
    double desired_angle = std::atan2(VfY, VfX);

    // Set cmd_vel
    double normVf = std::sqrt(VfX * VfX + VfY * VfY);
    cmd_vel.linear.x = std::min(V_MAX_DES, normVf);

    // Calculate difference in orientation
    double diff_angle = desired_angle - current_orientationr;

    // Normalize diff_angle to the range [-pi, pi]
    while (diff_angle > M_PI) diff_angle -= 2 * M_PI;
    while (diff_angle < -M_PI) diff_angle += 2 * M_PI;

    // Proportional Controller for rotation
    double rot_vel = 0.0;
    if (fabs(diff_angle) < ORI_ERROR) {
        rot_vel = K_ROT_MIN * V_MAX_ROT * diff_angle;
    } else {
        rot_vel = K_ROT_MAX * V_MAX_ROT * diff_angle;
		cmd_vel.linear.x=0;
    }
    cmd_vel.angular.z = rot_vel;

    return cmd_vel;
}
void callbackLaser(const sensor_msgs::LaserScan& most_intense) {
    static ros::Time last_decision_time = ros::Time::now(); // Track last decision time
    geometry_msgs::Twist cmd_vel; // Initialize cmd_vel
    if (isTheregoal) { // If there is a goal, we move toward it
        ros::Time current_time = ros::Time::now();

        if (ALGOR == 1) {
            if(avoidObstacle){ // If flag is active, wait for T_AVOID_OBS
                if ((current_time - last_decision_time).toSec() >= T_AVOID_OBS) {
                    avoidObstacle=false;
                    // Update the last decision time
                    last_decision_time = current_time;
                    if(DEBUG){
                        ROS_INFO("Normal State");
                    }
                }
                cmd_vel = algo1(most_intense);
                cmd_vel_pub.publish(cmd_vel);
            }else{
                cmd_vel = algo1(most_intense);
                // Update the last decision time
                last_decision_time = current_time;
                //Update velocities
                cmd_vel_pub.publish(cmd_vel);
            }
        }else if (ALGOR == 2) { // Potential fields algorithm
			// Check if enough time has passed for algo2 decisions
            if ((current_time - last_decision_time).toSec() * 1000 >= T_WAIT) {
                // Execute algo2 for potential fields algorithm
                cmd_vel = algo2(most_intense);
                // Update the last decision time
                last_decision_time = current_time;
                //Update velocities
                cmd_vel_pub.publish(cmd_vel);
            }
		}
        ros::spinOnce();

    }
}


/***
Print the odometry location.
***/
void callbackOdom(const nav_msgs::Odometry odom) {
/*
 The extracted values are:
 * - current_Xr: X-coordinate of the robot's current position.
 * - current_Yr: Y-coordinate of the robot's current position.
 * - current_z: Z-component of the robot's orientation quaternion.
 * - current_w: W-component of the robot's orientation quaternion.
 * - current_orientationr: The robot's orientation angle in radians.
 */
	current_Xr = odom.pose.pose.position.x;
	current_Yr = odom.pose.pose.position.y;
	current_z=odom.pose.pose.orientation.z;
	current_w=odom.pose.pose.orientation.w;
	current_orientationr=(2.0*atan2(current_z,current_w));

	if (isTheregoal) {
		double distance_to_goal = sqrt(pow(globalGoal.x - current_Xr, 2) + pow(globalGoal.y - current_Yr, 2));
		if (distance_to_goal < D_OBJ) {
			isTheregoal = false;
			ROS_INFO("Goal point achieved.");
		}
	}
}


void callbackmyGoal(const geometry_msgs::Point goal) {
	globalGoal.x=goal.x;
	globalGoal.y=goal.y;
	isTheregoal = true;

}


int main(int argc, char **argv) {



	ros::init(argc, argv, "moveRobot");
	ros::Time::init();

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

    // Parameter loading
    nh_private.param<int>("ID_ROBOT", ID_ROBOT, ID_ROBOT);
    nh_private.param<int>("ALGOR", ALGOR, ALGOR);
    nh_private.param<double>("CRIT_DIST", CRIT_DIST, CRIT_DIST);
    nh_private.param<double>("D_OBJ", D_OBJ, D_OBJ);
    nh_private.param<double>("V_MAX_DES", V_MAX_DES, V_MAX_DES);
    nh_private.param<double>("V_MAX_ROT", V_MAX_ROT, V_MAX_ROT);
    nh_private.param<double>("K_ROT_MIN", K_ROT_MIN, K_ROT_MIN);
    nh_private.param<double>("K_ROT_MAX", K_ROT_MAX, K_ROT_MAX);
    nh_private.param<double>("ORI_ERROR", ORI_ERROR, ORI_ERROR);
    nh_private.param<double>("T_AVOID_OBS", T_AVOID_OBS, T_AVOID_OBS);
    nh_private.param<double>("DIST_LEADER", DIST_LEADER, DIST_LEADER);
    nh_private.param<int>("ROBOT_ROL", ROBOT_ROL, ROBOT_ROL);
    nh_private.param<int>("ID_LEADER", ID_LEADER, ID_LEADER);
    nh_private.param<double>("W_1", W_1, W_1);
    nh_private.param<double>("W_2", W_2, W_2);
    nh_private.param<int>("T_WAIT", T_WAIT, T_WAIT);
    // ROS_INFO to print all parameters
    ROS_INFO("Robot Parameters:");
    ROS_INFO("ID_ROBOT: %d", ID_ROBOT);
    ROS_INFO("ALGOR: %d", ALGOR);
    ROS_INFO("CRIT_DIST: %.2f", CRIT_DIST);
    ROS_INFO("D_OBJ: %.2f", D_OBJ);
    ROS_INFO("V_MAX_DES: %.2f", V_MAX_DES);
    ROS_INFO("V_MAX_ROT: %.2f", V_MAX_ROT);
    ROS_INFO("K_ROT_MIN: %.2f", K_ROT_MIN);
    ROS_INFO("K_ROT_MAX: %.2f", K_ROT_MAX);
    ROS_INFO("ORI_ERROR: %.2f", ORI_ERROR);
    ROS_INFO("T_AVOID_OBS: %.2f", T_AVOID_OBS);
    ROS_INFO("DIST_LEADER: %.2f", DIST_LEADER);
    ROS_INFO("ROBOT_ROL: %d", ROBOT_ROL);
    ROS_INFO("ID_LEADER: %d", ID_LEADER);
    ROS_INFO("W_1: %.2f", W_1);
    ROS_INFO("W_2: %.2f", W_2);
    ROS_INFO("T_WAIT: %d", T_WAIT);

    // Dynamically construct topic names using ID_ROBOT
    std::stringstream ss;
    ss << "robot_" << ID_ROBOT;

    std::string robot_ns = ss.str(); // Example: "robot_1" for ID_ROBOT=1

    // Get the goal objective of the robot
    ros::Subscriber goal_sub = nh.subscribe("myGoals", 10, callbackmyGoal);

    // Publisher for robot velocity
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(robot_ns + "/cmd_vel", 10);

    // Subscriber for robot's odometry
    ros::Subscriber odom_sub = nh.subscribe(robot_ns + "/odom", 10, callbackOdom);

    // Subscriber for robot's laser scan
    ros::Subscriber laser_sub = nh.subscribe(robot_ns + "/base_scan_1", 1, callbackLaser);

	start = ros::Time::now();
	ros::spin();
}




