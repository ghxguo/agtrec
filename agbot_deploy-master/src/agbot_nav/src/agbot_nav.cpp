#include "PPController.h"
#include "ros/ros.h" 
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/NavSatFix.h"
#include "UTM.h"
#include <iostream>
using namespace std;


//define global variables
Point currentPosition;


//filename selection
const string file_name = "src/agbot_nav/src/waypoints_5_7_3.txt";

//callback functions

void pose_callback(const sensor_msgs::NavSatFix:: ConstPtr& msg)
{

	float x = msg->latitude;
	float y = msg->longitude;
	ROS_INFO("latitude: %f\tlongitude: %f", msg->latitude, msg->longitude);
	float iniX = 0;
	float iniY = 0;

	//iniX=DegToRad(x);
	//iniY=DegToRad(y);

	MapLatLonToXY(DegToRad(x), DegToRad(y), UTMCentralMeridian(17), iniX, iniY);

	currentPosition.x = iniX;
	currentPosition.y = iniY;

}


void heading_callback(const geometry_msgs::Point32:: ConstPtr& msg){

	currentPosition.inputHeading=msg->z;
    ROS_INFO("x: %f\ty: %f\tz: %f", msg->x, msg->y, msg->z);
	//cout<<currentPosition.inputHeading;
}


//execution function

void execute(int argc, char **agrv,PPController cntrl)
{
//setup ros publishers and subscribers
    
	double distance2Goal = 100000000;

	//initialize ppcontroller node
	ros::init(argc,agrv,"ppcontroller");
	
	//initialize publisher topics for testing
	// ros::init(argc,agrv,"steering_cmd");
	// ros::init(argc,agrv,"speed_setpoint");
	// ros::init(argc,agrv,"/current_goalpoint");
	
	//initialize subscriber topics for testing
	// ros::init(argc,agrv,"/fix");
	// ros::init(argc,agrv,"/novatel_imu");
	ros::NodeHandle nh; 
	auto fix = nh.subscribe("/fix", 500, pose_callback);
	auto imu = nh.subscribe("/novatel_imu", 500, heading_callback);
	//initialize publishers
	ros::Publisher pub_steering = nh.advertise<std_msgs::Float32>("steering_cmd",10);
	ros::Publisher pub_padel = nh.advertise<std_msgs::Float32>("speed_setpoint",10);
	ros::Publisher pub_goal = nh.advertise<geometry_msgs::Point32>("/current_goalpoint",10);
	
	//initialize test publishers for subscribers
	// ros::Publisher pub_heading = nh.advertise<geometry_msgs::Point32>("/novatel_imu",10);
	// ros::Publisher pub_pos = nh.advertise<sensor_msgs::NavSatFix>("/fix",10);
	
	ros::Rate loop_rate(10);

	//ros::Subscriber sub_heading = nh.subscribe("/novatel_imu",10,heading_callback);
	
	//initialize
	//1. Parameters (these are used for testing if the error is reducing properly)
	float error=0;
	float threshold=2.5;
	double velDouble;
	double deltaDouble;
	std_msgs::Float32 delta;
	std_msgs::Float32 vel;

	//2. Points
	Point goalPoint=cntrl.getwpList()[cntrl.getcurrWpIdx()];//not to be confused with "current_goalpoint"

	//3. Commands

	geometry_msgs::Point32 command;
	geometry_msgs::Point32 stationaryCommand;
	geometry_msgs::Point32 current_goalPoint;

	stationaryCommand.x=0;
	stationaryCommand.y=0;

	//stops when ros is shutdown
	while(ros::ok())
	{
		
		//compute the new Euclidean Error
		//geometry_msgs::Point32 current_goalPoint = geometry_msgs::Point32(goalPoint.x,goalPoint.y,0);
		current_goalPoint.x=goalPoint.x;
		current_goalPoint.y=goalPoint.y;

		cout <<"\nCurrent Index: "<< cntrl.getcurrWpIdx();

		pub_goal.publish(current_goalPoint);

		//Vehicule is in vicinity of goal point
		if(distance2Goal < 0.2)
		{
			//Update goal Point to next point in the waypoint list:
			cntrl.incrimentWpIdx();
			cout<< "/n Reached Waypoint # " <<cntrl.getcurrWpIdx();
			
			//checks to see if there are any more waypoints before updating goalpoint
			if (cntrl.getcurrWpIdx() < cntrl.getnPts()){
                goalPoint = cntrl.getwpList()[cntrl.getcurrWpIdx()];
			}
			else{
				cout<<"\n --- All Waypoints have been conquered! Mission Accomplished Mr Hunt !!! --- ";
				break;
			}
			
			cout<<"\nNew Goal is:\n"<<goalPoint.x<<goalPoint.y;
			
			cntrl.compute_steering_vel_cmds(currentPosition, velDouble, deltaDouble, distance2Goal);
			
			delta.data=-deltaDouble;
			vel.data=velDouble;
			
			pub_steering.publish(delta);
        	pub_padel.publish(vel);

			loop_rate.sleep();
		}
		ros::spin();
	}
}

/*testing junk
	geometry_msgs::Point32 testPoint;
	testPoint.x=0;
	testPoint.y=0;
	testPoint.z=1.324;

	//cout<<"Before loop good!";

	while(ros::ok()){
		std_msgs:: Float32 msg;
		//geometry_msgs:: Point32 msgP;
		float x=1.1;
		msg.data= x;
		//msgP.data=testPoint;
		pub_steering.publish(msg);
		//msg.data= 1.2;
		pub_heading.publish(testPoint);
		//msg.data= 1.3;
		pub_steering.publish(msg);
		ros::spinOnce();//used to trigger callback functions
		loop_rate.sleep();
	}
    */



int main(int argc, char **agrv)
{
    //cout << "HI!";
	AckermannVehicle mule = AckermannVehicle(2.065, 4.6, 2.2);
	PPController cntrl = PPController(0, mule.length, mule.minTurningRadius, mule.maximumVelocity);

	if (!cntrl.initialize(file_name))
	{
		return EXIT_FAILURE;
	}
    
	
	execute(argc,agrv,cntrl);

    return EXIT_SUCCESS;
}
