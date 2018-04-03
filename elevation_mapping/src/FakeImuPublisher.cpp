#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "fake_imu_publisher");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("/sensors/imu", 1000);

  ros::Rate loop_rate(400);

  while (ros::ok())
  {

    sensor_msgs::Imu msg;

    msg.angular_velocity.x = 0;
    msg.angular_velocity.y = 0;
    msg.angular_velocity.z = 0;

    msg.linear_acceleration.x = 0;
    msg.linear_acceleration.y = 0;
    msg.linear_acceleration.z = 0;

    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
