#include "ros/ros.h"
#include <ros/package.h>
#include <asclinic_pkg/LeftRightFloat32.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_publisher");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);
    ros::Rate loop_rate(10);

    asclinic_pkg::LeftRightFloat32 msg;
    msg.left = 10.0;
    msg.right = 10.0;

    while (ros::ok())
    {
        msg.left = 10;
        msg.right = 10;
        motor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}