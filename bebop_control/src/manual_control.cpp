/*
  _    _ _   _ _____    _    _    __      __  _               ____  
 | |  | | \ | |  __ \  | |  | |  /\ \    / / | |        /\   |  _ \ 
 | |  | |  \| | |__) | | |  | | /  \ \  / /  | |       /  \  | |_) |
 | |  | | . ` |  _  /  | |  | |/ /\ \ \/ /   | |      / /\ \ |  _ < 
 | |__| | |\  | | \ \  | |__| / ____ \  /    | |____ / ____ \| |_) |
  \____/|_| \_|_|  \_\  \____/_/    \_\/     |______/_/    \_\____/ 
                                                                    

                              __
                            .d$$b
                          .' TO$;\
                         /  : TP._;
                        / _.;  :Tb|
                       /   /   ;j$j
                   _.-"       d$$$$
                 .' ..       d$$$$;
                /  /P'      d$$$$P. |\
               /   "      .d$$$P' |\^"l
             .'           `T$P^"""""  :
         ._.'      _.'                ;
      `-.-".-'-' ._.       _.-"    .-"
    `.-" _____  ._              .-"
   -(.g$$$$$$$b.              .'
     ""^^T$$$P^)            .(:
       _/  -"  /.'         /:/;
    ._.'-'`-'  ")/         /;/;
 `-.-"..--""   " /         /  ;
.-" ..--""        -'          :
..--""--.-"         (\      .-(\
  ..--""              `-\(\/;`
    _.                      :
                            ;`-
                           :\
                           ;

Written by Ric Fehr
*/

// Set up two drones at different heights and opposite circle directions

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <fstream>

#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;

#define PI 3.141592653589793238462

/*
* Position is initialized at x=0, y=0, z=1 
* Button 5: x=0, y=0, z=1
* Button 6: x=1, y=0, z=1
* Button 7: x=0, y=1, z=1
* Button 9: x=0, y=0, z=2
* Button 1: Takeoff
* Button 3: Land
*/

// Struct for states data
struct states
{
	// Translational position in meters
	double pos_x, pos_y, pos_z;
	// Orientation position in quaternions
	double ori_x, ori_y, ori_z, ori_w;
	// Orientation position in euler angles
	double roll, pitch, yaw, yaw_RAD;
	// Error values for controller
	double error_x, error_y, error_z, error_yaw;
	// Instaneous velocity to be calculated
	double Inst_vel_x, Inst_vel_y, Inst_vel_z, Inst_vel_yaw;
	// Approx. velocity error values
	double velError_x, velError_y, velError_z, velError_yaw;
	// Approx. abstition error values
	double absitionError_x, absitionError_y, absitionError_z, absitionError_yaw;
	// Temp. variables to hold previous position for velocity calculation
	double prevPos_x, prevPos_y, prevPos_z, prevPos_yaw; 
	// Temp. variables to suppress cmd_vel_bebop1_ output to [-1,1]
	double cmd_x, cmd_y, cmd_z, cmd_yaw;
	// Rotation conversion variables for yaw orientation 
	double rot_cmd_x, rot_cmd_y;
} bebop1, bebop2, bebop3, bebop4, desired1, desired2, desired3, desired4;

int button_1;
int button_2;
int button_3;
int button_4;
int button_5;
int button_6;
int button_7;
float x_cmd, y_cmd, z_cmd, yaw_cmd;

// Get joystick controller info
void getJoy(const sensor_msgs::Joy::ConstPtr& button)
{
	button_1 = button->buttons[0];
	button_2 = button->buttons[1];
	button_3 = button->buttons[2];
	//lb, rb, lt, rt
	button_4 = button->buttons[4];
	button_5 = button->buttons[5];
	button_6 = button->buttons[6];
	button_7 = button->buttons[7];
	x_cmd = button->axes[1];
	y_cmd = button->axes[0];
	z_cmd = button->axes[3];
	yaw_cmd = button->axes[2];
}




int main(int argc, char** argv)
{
	bool translationControllerOn = true;
	bool orientationControllerOn = true;

	// Initialize ros
	ros::init(argc, argv, "bebop_control"); 

	// Node handle used
	ros::NodeHandle nh_; 
	
	// Initialized publishers
	// For bebop1
	ros::Publisher takeoff_pub_bebop1_;
	ros::Publisher land_pub_bebop1_;	
	ros::Publisher cmd_vel_pub_bebop1_;
	// For bebop2
	ros::Publisher takeoff_pub_bebop2_;
	ros::Publisher land_pub_bebop2_;	
	ros::Publisher cmd_vel_pub_bebop2_;
	// For bebop3
	ros::Publisher takeoff_pub_bebop3_;
	ros::Publisher land_pub_bebop3_;	
	ros::Publisher cmd_vel_pub_bebop3_;
	// For bebop4
	ros::Publisher takeoff_pub_bebop4_;
	ros::Publisher land_pub_bebop4_;	
	ros::Publisher cmd_vel_pub_bebop4_;
	// For bebop
	ros::Publisher takeoff_pub_bebop_;
	ros::Publisher land_pub_bebop_;	
	ros::Publisher cmd_vel_pub_bebop_;

	// Subscribe to joy node
	ros::Subscriber joy_controller = nh_.subscribe("/joy", 1000, getJoy);
	
	// Intitalize advertisers for each bebop
	// For bebop1
	takeoff_pub_bebop1_ = nh_.advertise<std_msgs::Empty>("/bebop_IP5/takeoff", 1000);
	land_pub_bebop1_ = nh_.advertise<std_msgs::Empty>("/bebop_IP5/land", 1000);
	cmd_vel_pub_bebop1_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP5/cmd_vel", 1);
	// For bebop2
	takeoff_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/takeoff", 1000);
	land_pub_bebop2_ = nh_.advertise<std_msgs::Empty>("/bebop_IP6/land", 1000);
	cmd_vel_pub_bebop2_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP6/cmd_vel", 1);
	// For bebop3
	takeoff_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP7/takeoff", 1000);
	land_pub_bebop3_ = nh_.advertise<std_msgs::Empty>("/bebop_IP7/land", 1000);
	cmd_vel_pub_bebop3_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP7/cmd_vel", 1);
	// For bebop4
	takeoff_pub_bebop4_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/takeoff", 1000);
	land_pub_bebop4_ = nh_.advertise<std_msgs::Empty>("/bebop_IP8/land", 1000);
	cmd_vel_pub_bebop4_ = nh_.advertise<geometry_msgs::Twist>("/bebop_IP8/cmd_vel", 1);
	// For bebop
	takeoff_pub_bebop_ = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 1000);
	land_pub_bebop_ = nh_.advertise<std_msgs::Empty>("/bebop/land", 1000);
	cmd_vel_pub_bebop_ = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);

	// Velocity data to be sent to individual drones and matlab
	geometry_msgs::Twist cmd_vel_bebop1_;
	geometry_msgs::Twist cmd_vel_bebop2_;
	geometry_msgs::Twist cmd_vel_bebop3_;
	geometry_msgs::Twist cmd_vel_bebop4_;
	geometry_msgs::Twist cmd_vel_bebop_;

	// std_msgs::Empty "takeoff" & "land"
	std_msgs::Empty msg_takeoff, msg_land; 

	ros::Rate loop_rate(100);
	
	while(nh_.ok())
	{

			// For bebop1
			cmd_vel_bebop1_.linear.x = x_cmd;
			cmd_vel_bebop1_.linear.y = y_cmd;
			cmd_vel_bebop1_.linear.z = z_cmd;
			// For bebop2
			cmd_vel_bebop2_.linear.x = x_cmd;
			cmd_vel_bebop2_.linear.y = y_cmd;
			cmd_vel_bebop2_.linear.z = z_cmd;
			// For bebop3
			cmd_vel_bebop3_.linear.x = x_cmd;
			cmd_vel_bebop3_.linear.y = y_cmd;
			cmd_vel_bebop3_.linear.z = z_cmd;
			// For bebop4
			cmd_vel_bebop4_.linear.x = x_cmd;
			cmd_vel_bebop4_.linear.y = y_cmd;
			cmd_vel_bebop4_.linear.z = z_cmd;
			// For bebop
			cmd_vel_bebop_.linear.x = x_cmd;
			cmd_vel_bebop_.linear.y = y_cmd;
			cmd_vel_bebop_.linear.z = z_cmd;
		

			// For bebop1 
			cmd_vel_bebop1_.angular.z = yaw_cmd;
			// For bebop2 
			cmd_vel_bebop2_.angular.z = yaw_cmd;
			// For bebop3 
			cmd_vel_bebop3_.angular.z = yaw_cmd;
			// For bebop4
			cmd_vel_bebop4_.angular.z = yaw_cmd;
			// For bebop
			cmd_vel_bebop_.angular.z = yaw_cmd;


		if (button_1 == 1)
		{
			printf("T A K E O F F\n");
			// For bebop1
			takeoff_pub_bebop1_.publish(msg_takeoff);
			// For bebop2
			takeoff_pub_bebop2_.publish(msg_takeoff);
			// For bebop3
			takeoff_pub_bebop3_.publish(msg_takeoff);
			// For bebop4
			takeoff_pub_bebop4_.publish(msg_takeoff);
			// For bebop
			takeoff_pub_bebop_.publish(msg_takeoff);
		}

		if (button_3 == 1)
		{
			translationControllerOn = false;
			orientationControllerOn = false;
			printf("L A N D\n");
			// For bebop1
			land_pub_bebop1_.publish(msg_land);
			// For bebop2
			land_pub_bebop2_.publish(msg_land);
			// For bebop3
			land_pub_bebop3_.publish(msg_land);
			// For bebop4
			land_pub_bebop4_.publish(msg_land);
			// For bebop
			land_pub_bebop_.publish(msg_land);
		}

		// Publish the assembled command
		// For bebop1
		cmd_vel_pub_bebop1_.publish(cmd_vel_bebop1_);
		// For bebop2
		cmd_vel_pub_bebop2_.publish(cmd_vel_bebop2_);
		// For bebop3
		cmd_vel_pub_bebop3_.publish(cmd_vel_bebop3_);
		// For bebop4
		cmd_vel_pub_bebop4_.publish(cmd_vel_bebop4_);
		// For bebop
		cmd_vel_pub_bebop_.publish(cmd_vel_bebop_);
		
		// Run subscriber update function
		ros::spinOnce();
		// Delay loop to keep 100 Hz rate
		loop_rate.sleep();
		}
	return 0;
}
