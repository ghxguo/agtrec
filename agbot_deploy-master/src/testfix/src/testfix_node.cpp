
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>
#include <random>
using namespace sensor_msgs;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "testfix_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 10);
  ros::Rate loop_rate(10);
  std::random_device r;
 
    // Choose a random mean between 1 and 6
    std::default_random_engine e(r());
    std::uniform_real_distribution<double> ul(-180, 180);
    std::uniform_real_distribution<double> ula(0, 90);
  while (ros::ok())
  {

    sensor_msgs::NavSatFix msg;
    msg.latitude = ula(e);
    msg.longitude = ul(e);
    chatter_pub.publish(msg);
    
    //std::cout <<  msg.latitude << "\t" << msg.longitude << "\n";

    ROS_INFO("latitude: %f\tlongitude: %f", msg.latitude, msg.longitude);
    //ROS_INFO("%f", msg.longitude);
    //ros::spin();

    loop_rate.sleep();

  }


  return 0;
}

