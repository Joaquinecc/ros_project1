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
/**
 * Robot Parameters and Flags
 */
int ID_ROBOT = 1;               // ID of the robot
int ALGOR = 1;                  // Algorithm selection: 1 (simple), 2 (potential fields)
double CRIT_DIST = 1.0;         // Critical distance for obstacle avoidance
double D_OBJ = 0.5;             // Distance threshold for goal achievement
double V_MAX_DES = 1.0;         // Maximum desired linear velocity
double V_MAX_ROT = 1.0;         // Maximum rotational velocity
double K_ROT_MIN = 0.1;         // Minimum rotation gain
double K_ROT_MAX = 0.25;        // Maximum rotation gain
double ORI_ERROR = 0.4;         // Orientation error tolerance
double T_AVOID_OBS = 2.0;       // Time to avoid obstacles
double DIST_LEADER = 2.0;       // Desired distance to leader
int ROBOT_ROL = 0;              // Role of the robot (e.g., leader, follower)
int ID_LEADER = 0;              // ID of the leader robot
double W_1 = 1.5;               // Weight for goal attraction
double W_2 = 3.0;               // Weight for obstacle repulsion
int T_WAIT = 150;               // Time to wait between algorithm decisions (ms)

/**
 * ROS Publisher and Global Variables
 */
ros::Publisher cmd_leader_vel_pub;      // Publisher for movement commands
ros::Publisher cmd_follower_vel_pub;      // Publisher for movement commands
ros::Time start;

geometry_msgs::Point goal_coord; // Global goal position
geometry_msgs::Point leader_coord; // Global goal position
double leader_orientationr;     // Current orientation in radians
geometry_msgs::Point follower_coord; // Global goal position
double follower_orientationr;     // Current orientation in radians

// State Flags
bool DEBUG = true;               // Debug mode flag
bool isTheregoal=false;


double calculateOrientationDifference(const geometry_msgs::Point goal,const geometry_msgs::Point robot_coords,const double robot_orientationr ) {

    // Calculate the desired angle to the goal point
    double desired_orientation = atan2(goal.y - robot_coords.y, goal.x - robot_coords.x);

    // Calculate the difference in orientation
    double orientation_difference = desired_orientation - robot_orientationr;

    // Normalize the orientation difference to the range [-π, π]
    while (orientation_difference > M_PI) {
        orientation_difference -= 2.0 * M_PI;
    }
    while (orientation_difference < -M_PI) {
        orientation_difference += 2.0 * M_PI;
    }

    return orientation_difference;
}

geometry_msgs::Twist algo1(const sensor_msgs::LaserScan& most_intense,const geometry_msgs::Point robot_coords, const double robot_orientationr,const geometry_msgs::Point goal_coord) {

    static bool avoid_obstacle = false; // Local flag to detect obstacles
    static ros::Time last_decision_time = ros::Time::now(); // Track last decision time


    //Set all to 0
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // Rotate until it find a free obstacle path
    int length = most_intense.ranges.size();
    for(int i=0; i<length; i++){
        if(most_intense.ranges[i] < CRIT_DIST) {
            if(DEBUG){
                ROS_INFO("OBSTACLES DETECTED");
            }
            avoid_obstacle= true;
            cmd_vel.angular.z = 1.0;
            last_decision_time = ros::Time::now();
            return cmd_vel;
        }
    }

    if(avoid_obstacle){ //Avoid obstacle for T_AVOID_OBS
        ros::Time current_time = ros::Time::now();

        if(DEBUG){
            ROS_INFO("AVOID OBSTACLES: Time elapsed = %f seconds", (current_time - last_decision_time).toSec());
        }
        if ((current_time - last_decision_time).toSec() >= T_AVOID_OBS) {
            avoid_obstacle=false;
            if(DEBUG){
                ROS_INFO("AVOID OBSTACLES DEACTIVATED");
            }
        }
        cmd_vel.linear.x = V_MAX_DES;
        return cmd_vel;
    }
    if(DEBUG){
        ROS_INFO("NORMAL OPERATIONS");
    }

    // Normal operation: move toward goal
    double diff_angle =  calculateOrientationDifference(goal_coord,robot_coords,robot_orientationr);
    // Proportional Controller for rotation
    double rot_vel = 0.0;
    if (fabs(diff_angle) < ORI_ERROR) {
        rot_vel = K_ROT_MIN * V_MAX_ROT * diff_angle;
        cmd_vel.linear.x = V_MAX_DES;
    } else {
        rot_vel = K_ROT_MAX * V_MAX_ROT * diff_angle;
    }
    cmd_vel.angular.z = rot_vel;
    return cmd_vel;

}

geometry_msgs::Twist algo2(const sensor_msgs::LaserScan& most_intense,const geometry_msgs::Point robot_coords, const double robot_orientationr,const geometry_msgs::Point goal_coord) {
    // Set all velocities to 0
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

	// Calculate Vobj (Go to Goal Behavior)
	double VobjX = goal_coord.x - robot_coords.x;
	double VobjY = goal_coord.y - robot_coords.y;
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
        if (di <most_intense.range_min || di > most_intense.range_max) {
            continue;
        }

        // Calculate obstacle local coordinates
        double obsX_local = di * cos(angle);
        double obsY_local = di * sin(angle);

        // Transform to global coordinates
        double obsX_global = obsX_local * cos(robot_orientationr) - obsY_local * sin(robot_orientationr) + robot_coords.x;
        double obsY_global = obsX_local * sin(robot_orientationr) + obsY_local * cos(robot_orientationr) + robot_coords.y;


        // Compute repulsion vector (global frame)
        double repX = robot_coords.x - obsX_global;
        double repY = robot_coords.y - obsY_global;

        // Compute magnitude of repulsion
        double normRep = std::sqrt(repX * repX + repY * repY);
        if (normRep > 0.0) { // Avoid division by zero
            repX /= normRep;
            repY /= normRep;

            // Scale repulsion by distance (if within CRIT_DIST)
            double magnitude = (di < CRIT_DIST) ? (CRIT_DIST - di) / CRIT_DIST : 0.0;
            VobsX += magnitude * repX;
            VobsY += magnitude * repY;
        }

    }

	// Calculate resulting vector Vf
    double VfX = W_1 * VobjX + W_2 * VobsX;
    double VfY = W_1 * VobjY + W_2 * VobsY;

    // Calculate desired orientation
    double desired_angle = std::atan2(VfY, VfX);

    // Set cmd_vel
    cmd_vel.linear.x = V_MAX_DES;

    // Calculate difference in orientation
    double diff_angle = desired_angle - robot_orientationr;

    // Normalize diff_angle to the range [-pi, pi]
    while (diff_angle > M_PI) diff_angle -= 2 * M_PI;
    while (diff_angle < -M_PI) diff_angle += 2 * M_PI;

    // Proportional Controller for rotation
    double rot_vel = 0.0;
    if (fabs(diff_angle) < ORI_ERROR) {
        rot_vel = K_ROT_MIN * V_MAX_ROT * diff_angle;
    } else {
        rot_vel = K_ROT_MAX * V_MAX_ROT * diff_angle;
		cmd_vel.linear.x=V_MAX_DES*0.3;
    }
    cmd_vel.angular.z = rot_vel;

    return cmd_vel;
}

void callbackLaserLeader(const sensor_msgs::LaserScan& most_intense) {
    static ros::Time last_decision_time = ros::Time::now(); // Track last decision time
    geometry_msgs::Twist cmd_vel; // Initialize cmd_vel
    if (isTheregoal) { // If there is a goal, we move toward it

        if (ALGOR == 1) {
                cmd_vel = algo1(most_intense,leader_coord,leader_orientationr,goal_coord);
                cmd_leader_vel_pub.publish(cmd_vel);
        }else if (ALGOR == 2) { // Potential fields algorithm
             ros::Time current_time = ros::Time::now();
			// Check if enough time has passed for algo2 decisions
            if ((current_time - last_decision_time).toSec() * 1000 >= T_WAIT) {
                // Execute algo2 for potential fields algorithm
                cmd_vel = algo2(most_intense,leader_coord,leader_orientationr,goal_coord);
                // Update the last decision time
                last_decision_time = current_time;
                //Update velocities
                cmd_leader_vel_pub.publish(cmd_vel);
            }
		}
        ros::spinOnce();

    }
}

void callbackLaserFollower(const sensor_msgs::LaserScan& most_intense) {
    if (ROBOT_ROL == 0 && !isTheregoal) return;
    static ros::Time last_decision_time = ros::Time::now(); // Track last decision time
    geometry_msgs::Twist cmd_vel; // Initialize cmd_vel
    double distance_to_leader = sqrt(pow(leader_coord.x - follower_coord.x, 2) + pow(leader_coord.y - follower_coord.y, 2));
    if(DEBUG){
        ROS_INFO("DISTANCE TO LEADER: %.2f", distance_to_leader);
    }
    if(distance_to_leader>DIST_LEADER){//Check distance with the leader
        if (ALGOR == 1) {
                cmd_vel = algo1(most_intense,follower_coord,follower_orientationr,leader_coord);
                cmd_follower_vel_pub.publish(cmd_vel);
                ros::spinOnce();
        }else if (ALGOR == 2) { // Potential fields algorithm
            ros::Time current_time = ros::Time::now();
            // Check if enough time has passed for algo2 decisions
            if ((current_time - last_decision_time).toSec() * 1000 >= T_WAIT) {
                // Execute algo2 for potential fields algorithm
                cmd_vel = algo2(most_intense,follower_coord,follower_orientationr,leader_coord);
                // Update the last decision time
                last_decision_time = current_time;
                //Update velocities
                cmd_follower_vel_pub.publish(cmd_vel);
                ros::spinOnce();
            }
        }
    }else{ //To close to leader, stop
        ros::Time current_time = ros::Time::now();
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        // Update the last decision time
        last_decision_time = current_time;
        //Update velocities
        cmd_follower_vel_pub.publish(cmd_vel);
        ros::spinOnce();

    }

}


void callbackOdomLeader(const nav_msgs::Odometry odom) {
	leader_coord.x = odom.pose.pose.position.x;
	leader_coord.y = odom.pose.pose.position.y;
	double current_z=odom.pose.pose.orientation.z;
	double current_w=odom.pose.pose.orientation.w;
	leader_orientationr=(2.0*atan2(current_z,current_w));

	if (isTheregoal) {
		double distance_to_goal = sqrt(pow(goal_coord.x - leader_coord.x, 2) + pow(goal_coord.y - leader_coord.y, 2));
		if (distance_to_goal < D_OBJ) {
			isTheregoal = false;
			ROS_INFO("Goal point achieved.");
		}
	}
}
void callbackOdomFollower(const nav_msgs::Odometry odom) {
    if (ROBOT_ROL == 0 && !isTheregoal) return;
	follower_coord.x = odom.pose.pose.position.x;
	follower_coord.y = odom.pose.pose.position.y;
	double current_z=odom.pose.pose.orientation.z;
	double current_w=odom.pose.pose.orientation.w;
	follower_orientationr=(2.0*atan2(current_z,current_w));
}

/**
 * @brief Goal callback function.
 *
 * Sets the global goal position and activates goal-seeking behavior.
 *
 * @param goal Goal position data.
 */
void callbackmyGoal(const geometry_msgs::Point goal) {
	goal_coord.x=goal.x;
	goal_coord.y=goal.y;
	isTheregoal = true;

}

/**
 * @brief Main function.
 *
 * Initializes ROS nodes, publishers, and subscribers, and starts the spin loop.
 */
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



    // Dynamically construct topic names using Leader id
    std::stringstream ss;
    ss << "robot_" << (ROBOT_ROL == 0 ? ID_ROBOT : ID_LEADER);
    std::string robot_leader = ss.str();
    ROS_INFO("robot_leader: %s", robot_leader.c_str());
    ros::Subscriber goal_sub = nh.subscribe("myGoals", 10, callbackmyGoal);
    cmd_leader_vel_pub = nh.advertise<geometry_msgs::Twist>(robot_leader + "/cmd_vel", 10);
    ros::Subscriber odom_sub = nh.subscribe(robot_leader + "/odom", 10, callbackOdomLeader);
    ros::Subscriber laser_sub = nh.subscribe(robot_leader + "/base_scan_1", 1, callbackLaserLeader);

    //Follower
    std::stringstream ss_follower;
    ss_follower << "robot_" << ID_ROBOT;
    std::string robot_follower = ss_follower.str();
    ROS_INFO("robot_follower: %s", robot_follower.c_str());
    cmd_follower_vel_pub = nh.advertise<geometry_msgs::Twist>(robot_follower + "/cmd_vel", 10);
    ros::Subscriber odom_sub_follower = nh.subscribe(robot_follower + "/odom", 10, callbackOdomFollower);
    ros::Subscriber laser_sub_follower = nh.subscribe(robot_follower + "/base_scan_1", 1, callbackLaserFollower);


	start = ros::Time::now();
	ros::spin();
}
