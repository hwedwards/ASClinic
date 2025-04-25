#include <ros/ros.h>
#include "asclinic_pkg/PoseCovar.h"
#include "asclinic_pkg/LeftRightFloat32.h"
#include <cmath>

// Constants
const float LINEAR_SPEED = 0.5;    // in m/s
const float ANGULAR_SPEED = 30.0;  // in degrees/s
const float POSITION_TOLERANCE = 0.1; // in meters
const float ANGLE_TOLERANCE = 5.0;    // in degrees

// Variables
ros::Publisher motor_duty_cycle_publisher;
float current_x = 0.0;
float current_y = 0.0;
float current_phi = 0.0;

// Helper function to publish motor commands
void publishMotorCommand(float left_speed, float right_speed) {
    asclinic_pkg::LeftRightFloat32 duty_cycle_msg;
    duty_cycle_msg.left = left_speed;
    duty_cycle_msg.right = right_speed;
    motor_duty_cycle_publisher.publish(duty_cycle_msg);
}

// Callback for reference trajectory
void referenceCallback(const asclinic_pkg::PoseCovar& msg) {
    float target_x = msg.x;
    float target_y = msg.y;
    float target_phi = msg.phi;

    // Compute errors
    float error_x = target_x - current_x;
    float error_y = target_y - current_y;
    float error_phi = target_phi - current_phi;

    // Normalize angle error to [-180, 180]
    error_phi = fmod(error_phi + 180.0, 360.0) - 180.0;

    // Check if within tolerance
    if (std::abs(error_x) < POSITION_TOLERANCE && std::abs(error_y) < POSITION_TOLERANCE && std::abs(error_phi) < ANGLE_TOLERANCE) {
        publishMotorCommand(0.0, 0.0);
        ROS_INFO("Target reached: x=%.2f, y=%.2f, phi=%.2f", target_x, target_y, target_phi);
        return;
    }

    // Simple proportional control for linear and angular velocity
    float linear_velocity = LINEAR_SPEED * sqrt(error_x * error_x + error_y * error_y);
    float angular_velocity = ANGULAR_SPEED * error_phi / 180.0;

    // Convert to left and right wheel speeds
    float left_speed = linear_velocity - angular_velocity;
    float right_speed = linear_velocity + angular_velocity;

    publishMotorCommand(left_speed, right_speed);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_tracker");
    ros::NodeHandle nh;

    motor_duty_cycle_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);
    ros::Subscriber reference_subscriber = nh.subscribe("/reference_trajectory", 10, referenceCallback);

    ros::spin();

    return 0;
}