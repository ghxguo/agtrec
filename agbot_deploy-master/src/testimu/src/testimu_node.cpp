
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include <sstream>
#include <random>
using namespace geometry_msgs;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "testimu_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<Point32>("novatel_imu", 10);
  ros::Rate loop_rate(10);
  std::random_device r;
 
    // Choose a random mean between 1 and 6
    std::default_random_engine e(r());
    std::uniform_real_distribution<float> u(0,1);
  while (ros::ok())
  {

    Point32 msg;
    msg.x = u(e);
    msg.y = u(e);
    msg.z = u(e);
    chatter_pub.publish(msg);

    ROS_INFO("x: %f\ty: %f\tz: %f", msg.x, msg.y, msg.z);
    //ROS_INFO("%f", msg.longitude);
    //ros::spin();

    loop_rate.sleep();

  }


  return 0;
}

