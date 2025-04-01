#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "fun_publisher");
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher fun_publisher = nh.advertise<std_msgs::String>("fun_police", 10);

    // Set the loop rate
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // Create and populate the message
        std_msgs::String msg;
        msg.data = "this is soo fun";

        // Publish the message
        fun_publisher.publish(msg);

        // Log the message
        ROS_INFO("%s", msg.data.c_str());

        // Sleep for the remaining time until we hit the loop rate
        loop_rate.sleep();
    }

    return 0;
}