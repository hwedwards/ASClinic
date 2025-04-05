#include "ros/ros.h"
#include <ros/package.h>
#include "asclinic_pkg/PoseSeqs.h"
#include "asclinic_pkg/LeftRightInt32.h"

// whenever a message comes to the 'testtopic' topic, this callback is executed
void callbackFn(const asclinic_pkg::LeftRightInt32 &msg)
{
    // Respond to the message received
    ROS_INFO_STREAM("Message received with data: " << msg.left);
}

int main(int argc, char *argv[])
{
    // Initialise the node
    ros::init(argc, argv, "odometer");

    // Initialise a node handle to the group namespace
    std::string ns_for_group = ros::this_node::getNamespace();
    // ROS_INFO("Namespace: %s", ns.c_str());
    ros::NodeHandle nh_for_group(ns_for_group);

    ros::Rate loop_rate(2);

    // Subscribe to encoder_counts
    ros::Subscriber subscriber = nh_for_group.subscribe("/asc/encoder_counts", 1, callbackFn);

    // Initialise a publisher
    ros::Publisher m_publisher = nh_for_group.advertise<asclinic_pkg::PoseSeqs>("Pose", 10);
    asclinic_pkg::PoseSeqs pose;
    pose.x = 3.141592;
    pose.y = 2.71828;
    pose.phi = 1.618;
    pose.seq_k = 0;
    pose.seq_aruco = 0;
    m_publisher.publish(pose);
    // ROS_INFO("Message = %d", msg);

    // Spin as a single-threaded node
    // ros::Rate loop_rate(10);

    // Spin at a specific rate, 2Hz here
    while (ros::ok())
    {
        // ROS_INFO("Node is running, message is %d", msg.data);
        m_publisher.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}