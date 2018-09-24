#include "utilities.h"
#include "ros/ros.h" //this needs to be included for publisher/subscriber to work


//define global variables
Point currentPosition;


//various ways to initizialize file_name

//string file_name;
//file_name = rospy.get_param("/file_name");
//ros::param:get("/file_name",file_name);
string file_name="waypoints_5_7_3.txt";

//Initialize function definition:
PPController initialize(){

//Create objects for AckermannVehicle and Pure Pursuit controller
AckermannVehicle mule = AckermannVehicle(2.065,4.6,2.2);
PPController cntrl = PPController(0,mule.length,mule.minTurningRadius,mule.maximumVelocity);

cntrl.initialize(file_name);

return cntrl;
}

//execution function



/*
void execute(PPController cntrl){

//setup ros publishers and subscribers
    ros::NodeHandle nh; //this can't work unless we include ros/ros.h

    Publisher pub_steering = nh.publish('steering_cmd', Float32, queue_size =10);
}
*/
int main()
{
    //PPController cntrl = initialize();
    //execute(cntrl);
	VectorXd a(5);
	a.setZero();
	/*a(1, 3) = 1;
	a(1, 2) = 2;
	VectorXd b = a.Unit(1);
	cout << b;*/
    return 0;
}
