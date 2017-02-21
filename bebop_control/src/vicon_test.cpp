#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>

#define PI 3.141592645

struct states
{
	double pos_x;
	double pos_y;
	double pos_z;
	double ori_x;
	double ori_y;
	double ori_z;
	double ori_w;
	double roll;
	double pitch;
	double yaw;
	double yaw_RAD;
} bebop;


// Get position info from mocap for bebop1
void getPos(const geometry_msgs::TransformStamped::ConstPtr& pos)
{
	bebop.pos_x = pos->transform.translation.x;
	bebop.pos_y = pos->transform.translation.y;
	bebop.pos_z = pos->transform.translation.z;
	bebop.ori_x = pos->transform.rotation.x;
	bebop.ori_y = pos->transform.rotation.y;
	bebop.ori_z = pos->transform.rotation.z;
	bebop.ori_w = pos->transform.rotation.w;

	tf::Quaternion q(pos->transform.rotation.x,pos->transform.rotation.y,pos->transform.rotation.z,pos->transform.rotation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(bebop.roll, bebop.pitch, bebop.yaw_RAD);
	bebop.yaw = bebop.yaw_RAD*(180/PI);
}

int main(int argc, char** argv)
{
	// Initialize ros
	ros::init(argc, argv, "bebop_control");
	
	// Node handle used
	ros::NodeHandle nh_;
	
	// Get data from vicon
	ros::Subscriber subPoseActual1 = nh_.subscribe("<insert_topic_here>", 1000, getPos); 
	
	ros::Rate loop_rate(100);
	
	while(nh_.ok())
	{
		printf("bebop.pos_x|%lf| bebop.pos_y|%lf| bebop.pos_z|%lf| bebop.roll|%lf| bebop.pitch|%lf| bebop.yaw|%lf|\n",  bebop.pos_x, bebop.pos_y, bebop.pos_z, bebop.roll, bebop.pitch, bebop.yaw);
		// Run subscriber update function
		ros::spinOnce();
		// Delay loop to keep 100 Hz rate
		loop_rate.sleep();
	}
	
	return 0;
}
