/*
 * CPSC 481 Assignment 2 
 * Box Turtle autonomously navigate on a rectangle loop
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;

const double PI = 3.14159265359;
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	cout<<"\n******START******"<<endl;
	ros::Rate loop(0.5);

	move(1, 4, true);
	loop.sleep();
	rotate(degrees2radians(10), degrees2radians(90), true);
	
	move(1, 2, true);
	loop.sleep();
	rotate(degrees2radians(10), degrees2radians(90), true);
	
	move(1, 4, true);
	loop.sleep();
	rotate(degrees2radians(10), degrees2radians(90), true);
	
	move(1, 2, true);
	loop.sleep();
	rotate(degrees2radians(10), degrees2radians(90), true);
	

	cout<<"\n******END******"<<endl;
	return 0;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}


void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}