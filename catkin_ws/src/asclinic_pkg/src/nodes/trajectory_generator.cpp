#include <ros/ros.h>
#include "asclinic_pkg/LeftRightFloat32.h"

// Constants
const float ANGULAR_SPEED = 45.0;  // in degrees/s
const float TURN_DURATION = 4.0;  // in seconds for 180 degrees

// Variables
ros::Publisher motor_duty_cycle_publisher;
ros::Time start_time;

// Helper function to publish motor commands
void publishMotorCommand(float left_speed, float right_speed) {
    asclinic_pkg::LeftRightFloat32 motor_command;
    motor_command.left = left_speed;
    motor_command.right = right_speed;
    motor_duty_cycle_publisher.publish(motor_command);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    motor_duty_cycle_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);

    start_time = ros::Time::now();
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        float elapsed_time = (current_time - start_time).toSec();

        if (elapsed_time <= TURN_DURATION) {
            // Turn right to reach 180 degrees in 4 seconds
            publishMotorCommand(-ANGULAR_SPEED, ANGULAR_SPEED);
        } else {
            // Stop
            publishMotorCommand(0.0, 0.0);
            ROS_INFO("Trajectory complete.");
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}