#include <ros/ros.h>
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/PoseCovar.h"

// Constants
const float LINEAR_SPEED = 300;    // in mm/s

// Variables
ros::Publisher motor_reference_position;
ros::Time start_time;

// Helper function to publish motor commands

// Helper function to publish position commands
void publishPositionCommand(float x, float y, float phi) {
    asclinic_pkg::PoseCovar position_command;
    position_command.x = x;
    position_command.y = y;
    position_command.phi = phi;
    motor_reference_position.publish(position_command);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    motor_reference_position = nh.advertise<asclinic_pkg::PoseCovar>("/reference_trajectory", 10);

    start_time = ros::Time::now();
    ros::Rate loop_rate(10); // 10 Hz

    float current_x = 0.0;
    const float target_x = 10000;

    while (ros::ok() && current_x < target_x) {
        ros::Time current_time = ros::Time::now();
        float elapsed_time = (current_time - start_time).toSec();

        // Update position along x-axis
        current_x += LINEAR_SPEED / 10.0; // Increment x based on speed and loop rate
        ROS_INFO("Current x: %f", current_x);
        publishPositionCommand(current_x, 0.0, 0.0);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}