#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/LeftRightFloat32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

//include asclinic message type
#include "asclinic_pkg/LeftRightFloat32.h"

// Declare "member" variables
ros::Timer m_timer_for_publishing;
ros::Publisher m_publisher;

// Declare the function prototypes
void timerCallback(const ros::TimerEvent&);

// Implement the timer callback function
void timerCallback(const ros::TimerEvent&)
{
    static uint counter = 0;
    counter += 2;
    // Build and publish a message
    asclinic_pkg::LeftRightFloat32 msg;
    msg.data = {left: 10.1, right: 10.1};
    m_publisher.publish(msg);
    std::cout << "Publishing message: " << msg << endl;
}

int main(int argc, char* argv[])
{
    // Initialise the node
    ros::init(argc, argv, "motor_pwm_control");
    // Start the node by initialising a node handle
    ros::NodeHandle nh("~");
    // Display the namespace of the node handle
    ROS_INFO_STREAM("[PUBLISHER  CPP NODE] namespace of nh = " << nh.getNamespace());
    // Initialise a node handle to the group namespace
    std::string ns_for_group = ros::this_node::getNamespace();
    ros::NodeHandle nh_for_group(ns_for_group);
    // Initialise a publisher relative to the group namespace
    uint32_t queue_size = 10;
    bool should_latch = false;
    m_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("set_motor_duty_cycle", queue_size, should_latch);
    // Initialise a timer
    float timer_delta_t_in_seconds = 0.5;
    m_timer_for_publishing = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, false);
    // Spin as a single-threaded node
    ros::spin();
    // Main has ended, return 0
    return 0;
}