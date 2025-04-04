#include "ros/ros.h"
#include <ros/package.h>
#include <math.h>
#include "asclinic_pkg/PoseSeqs.h"
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/LeftRightFloat32.h"

#define THRESHOLD_DISTANCE 1000 // in mm

// use global variables, static means scope is limited to this script
static float d = 0, pose_x = 0, pose_y = 0;
static bool first_message = true; // Flag to handle first message separately
static int leftcount = 0, rightcount = 0;

// whenever a message comes to the '/asc/encoder_counts' topic, this callback is executed
void calculate_distance(const asclinic_pkg::PoseSeqs &msg)
{
    // Skip distance calculation for the first message
    if (first_message)
    {
        pose_x = msg.x;
        pose_y = msg.y;
        first_message = false;
        ROS_INFO("Initial pose at: (%f, %f)", pose_x, pose_y);
        return; // Skip distance calculation on first message
    }

    // Calculate the distance based on the change in position
    d += sqrt(pow(msg.x - pose_x, 2) + pow(msg.y - pose_y, 2));
    pose_x = msg.x;
    pose_y = msg.y;
}

void countticks(const asclinic_pkg::LeftRightInt32 &msg)
{
    leftcount += msg.left;
    rightcount += msg.right;
}

int main(int argc, char *argv[])
{
    d = 0;
    pose_x = 0;
    pose_y = 0;
    leftcount = 0;
    rightcount = 0;

    // Initialise the node
    ros::init(argc, argv, "odomtester");

    // Initialise a node handle to the group namespace
    std::string ns_for_group = ros::this_node::getNamespace();
    ros::NodeHandle nh_for_group(ns_for_group);

    ros::Rate loop_rate(10);

    // Subscribe to /Pose
    ros::Subscriber pose_subscriber = nh_for_group.subscribe("Pose", 10, calculate_distance);
    // Subscribe to /asc/encoder_counts
    ros::Subscriber encodersubscriber = nh_for_group.subscribe("/asc/encoder_counts", 1, countticks);

    // Initialise a publisher
    ros::Publisher m_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);
    asclinic_pkg::LeftRightFloat32 dutycycle;

    ros::Duration(1).sleep();
    dutycycle.left = 20;
    dutycycle.right = 20;
    dutycycle.seq_num = 0;

    m_publisher.publish(dutycycle);
    ROS_INFO("Duty cycle = %f, %f, Distance travelled = %f", dutycycle.left, dutycycle.right, d);
    ROS_INFO("To begin, ticks counted left: %d, right: %d", leftcount, rightcount);
    /* Alternatively if you want to run it for a certain time, get rid of the while loop and do this:
        // Wait 1 seconds
        ros::Duration(1).sleep();
        dutycycle.left = 0;
        dutycycle.right = 0;

        // Then publish final message

        m_publisher.publish(dutycycle);
        ROS_INFO("Message = %f, %f, %d", dutycycle.left, dutycycle.right, dutycycle.seq_num);
    */

    // Spin at a specific rate, 10Hz here
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("Node is running, Distance travelled: %f", d);
        if (d > THRESHOLD_DISTANCE)
        {
            dutycycle.left = 0;
            dutycycle.right = 0;
            m_publisher.publish(dutycycle);
            ROS_INFO("Duty cycle = %f, %f, Distance travelled = %f", dutycycle.left, dutycycle.right, d);
            ROS_INFO("Final pose at: (%f, %f)", pose_x, pose_y);
            ROS_INFO("Total ticks counted left: %d, right: %d", leftcount, rightcount);
            break;
        }
        loop_rate.sleep();
    }

    return 0;
}