#include <ros/ros.h>
#include "asclinic_pkg/PoseCovar.h"
#include "asclinic_pkg/LeftRightFloat32.h"
#include <cmath>

// rostopic pub /reference_trajectory asclinic_pkg/PoseCovar "{x: 0, y: 0, phi: 90.0}"
// Constants
const float LINEAR_SPEED = 0.5;    // in m/s
const float ANGULAR_SPEED = 30.0;  // in degrees/s
const float POSITION_TOLERANCE = 0.1; // in meters
const float ANGLE_TOLERANCE = 5.0;    // in degrees
const float WHEEL_RADIUS = 0.072; // in meters
const float WHEEL_BASE = 0.215/2; // in meters
const int RPM_TO_DEG = 6; // conversion factor 
// Variables
ros::Publisher velocity_reference_publisher;

namespace RobotState {
    float current_x = 0.0;
    float current_y = 0.0;
    float current_phi = 0.0;
}

// Function to publish motor commands
void publishMotorCommand(float left_speed, float right_speed) {
    asclinic_pkg::LeftRightFloat32 motor_command;
    // NOTE: May need to convert from degrees to RPM
    motor_command.left = left_speed/RPM_TO_DEG;
    motor_command.right = right_speed/RPM_TO_DEG;
    velocity_reference_publisher.publish(motor_command);
}

// Callback for reference trajectory
void referenceCallback(const asclinic_pkg::PoseCovar& msg) {
    float target_x = msg.x;
    float target_y = msg.y;
    float target_phi = msg.phi;

    // Compute errors
    float error_x = target_x - RobotState::current_x;
    float error_y = target_y - RobotState::current_y;
    float error_phi = target_phi - RobotState::current_phi;

    // Check if within tolerance
    if (std::abs(error_x) < POSITION_TOLERANCE && std::abs(error_y) < POSITION_TOLERANCE && std::abs(error_phi) < ANGLE_TOLERANCE) {
        publishMotorCommand(0.0, 0.0);
        ROS_INFO("Target reached: x=%.2f, y=%.2f, phi=%.2f", target_x, target_y, target_phi);
        return;
    }

    // Simple proportional control for linear and angular velocity
    float linear_velocity = 0; // LINEAR_SPEED * sqrt(error_x * error_x + error_y * error_y);
    float angular_velocity[2] = {WHEEL_BASE / WHEEL_RADIUS, -WHEEL_BASE / WHEEL_RADIUS}; // Array for angular velocity

    // Convert to left and right wheel speeds
    float left_speed = linear_velocity + angular_velocity[1] * error_phi;
    float right_speed = linear_velocity + angular_velocity[0] * error_phi;

    publishMotorCommand(left_speed, right_speed);
}

// Callback to update the robot's current state
void stateUpdateCallback(const asclinic_pkg::PoseCovar& msg) {
    RobotState::current_x = msg.x;
    RobotState::current_y = msg.y;
    RobotState::current_phi = msg.phi;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_tracker");
    ros::NodeHandle nh;

    velocity_reference_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/set_reference", 10);
    ros::Subscriber reference_subscriber = nh.subscribe("/reference_trajectory", 10, referenceCallback);
    ros::Subscriber state_subscriber = nh.subscribe("/current_state", 10, stateUpdateCallback);

    ros::spin();

    return 0;
}