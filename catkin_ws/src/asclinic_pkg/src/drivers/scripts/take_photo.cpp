#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/UInt32.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "take_photo");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<std_msgs::UInt32>("/asc/request_save_image", 10);
    ros::Rate loop_rate(1);

    std_msgs::UInt32 msg;

    while (ros::ok())
    {
        msg.data = 1;
        motor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}