
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"

void steering_cmdCB(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("%f",msg->data);
}

void speed_setpointCB(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("%f",msg->data);
}

void current_goalpointCB(const geometry_msgs::Point32::ConstPtr& msg)
{
    ROS_INFO("x: %f\ty: %f\tz: %f", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "testsub_node");


  ros::NodeHandle n;


  ros::Subscriber sub1 = n.subscribe("/steering_cmd", 500, steering_cmdCB);
  ros::Subscriber sub2 = n.subscribe("/speed_setpoint", 500, speed_setpointCB);
  ros::Subscriber sub3 = n.subscribe("/current_goalpoint", 500, current_goalpointCB);

  ros::spin();

  return 0;
}
