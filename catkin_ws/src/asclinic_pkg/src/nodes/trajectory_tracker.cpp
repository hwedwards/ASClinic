#include <ros/ros.h>
#include "asclinic_pkg/PoseCovar.h"
#include "asclinic_pkg/LeftRightFloat32.h"
#include <cmath>
#include "std_msgs/String.h"
// rostopic pub /reference_trajectory asclinic_pkg/PoseCovar "{x: 0, y: 0, phi: 90.0}"
// Constants
const float LINEAR_SPEED = 0.5;    // in m/s
const float ANGULAR_SPEED = 30.0;  // in degrees/s
const float POSITION_TOLERANCE = 0.1; // in meters
const float ANGLE_TOLERANCE = 5.0;    // in degrees
const float WHEEL_RADIUS = 0.072; // in meters
const float WHEEL_BASE = 0.215/2; // in meters
const int RPM_TO_DEG = 6; // conversion factor 
const float K_angular = 60; // Proportional gain for angular velocity control
const float Kd_angular = 25; // Derivative gain for angular velocity control
const float K_line_progress = 50; // Proportional gain for line progress control
const float Kp_line_deviation =3; //0.2 // Proportional gain for line deviation control
const float Kd_line_deviation = 2.5; 
const float Ki = 0.002; // Integral gain for line deviation control
ros::Publisher velocity_reference_publisher;
ros::Subscriber driving_state_subscriber;
namespace DrivingState {
    std::string current_state = "FORWARD"; // Default state
}

void drivingStateCallback(const std_msgs::String& msg) {
    DrivingState::current_state = msg.data;
}

namespace RobotState {
    float current_x = 0.0;
    float current_y = 0.0;
    float current_phi = 0.0;
    float w = 0.0; // angular velocity
}
namespace ReferenceTrajectory {
    float target_x = 0.0;
    float target_y = 0.0;
    float target_phi = 0.0;
}
namespace Error {
    float error_x = 0.0;
    float error_y = 0.0;
    float error_phi = 0.0; 
    float integral_error_y = 0.0; // Integral of the lateral error
    const float INTEGRAL_MAX = 100.0; // Maximum limit for integral term
    const float INTEGRAL_MIN = -100.0; // Minimum limit for integral term
}

// Function to publish motor commands
void publishMotorCommand(float left_speed, float right_speed) {
    asclinic_pkg::LeftRightFloat32 motor_command;
    // NOTE: May need to convert from degrees to RPM
    motor_command.left = left_speed/RPM_TO_DEG;
    motor_command.right = right_speed/RPM_TO_DEG;
    ROS_INFO("Publishing motor command - Left: %f, Right: %f", motor_command.left, motor_command.right);
    velocity_reference_publisher.publish(motor_command);
}

// Callback for reference trajectory
void referenceCallback(const asclinic_pkg::PoseCovar& msg) {
    ReferenceTrajectory::target_x = msg.x;
    ReferenceTrajectory::target_y = msg.y;
    ReferenceTrajectory::target_phi = msg.phi;
}
double signedAngleDiffDeg(double ref_deg, double meas_deg) {
    // raw difference
    double diff = ref_deg - meas_deg;
    // wrap to (−180, +180]
    diff = std::fmod(diff + 180.0, 360.0);
    if (diff < 0.0)
        diff += 360.0;
    return diff - 180.0;
}
double pureRotationController(){
    float error_phi = signedAngleDiffDeg(ReferenceTrajectory::target_phi, RobotState::current_phi);
    float dphi = (error_phi - Error::error_phi) / 0.1;
    Error::error_phi = error_phi;
    return (error_phi * K_angular + Kd_angular * dphi);
}
double lineDeviationController() {
    float error_y = ReferenceTrajectory::target_y - RobotState::current_y;
    ROS_INFO("error_y: %f", error_y);

    // Update the integral of the error
    Error::integral_error_y += error_y * 0.1; // Assuming a fixed time step of 0.1 seconds

    // Apply anti-windup by clamping the integral term
    if (Error::integral_error_y > Error::INTEGRAL_MAX) {
        Error::integral_error_y = Error::INTEGRAL_MAX;
    } else if (Error::integral_error_y < Error::INTEGRAL_MIN) {
        Error::integral_error_y = Error::INTEGRAL_MIN;
    }
    ROS_INFO("integral_error_y: %f", Error::integral_error_y);
    // Calculate the derivative of the error
    float derivative_error_y = (error_y - Error::error_y) / 0.1; // Assuming a fixed time step of 0.1 seconds

   float w = Kp_line_deviation * error_y + Kd_line_deviation * derivative_error_y + Ki * Error::integral_error_y;
    // Update the previous error
    Error::error_y = error_y;

    ROS_INFO("w: %f", w);
    return w; 
}
// Callback to update the robot's current state
void stateUpdateCallback(const asclinic_pkg::PoseCovar& msg) {
    RobotState::current_x = msg.x;
    RobotState::current_y = msg.y;
    RobotState::current_phi = msg.phi;
    // Compute errors
    float error_x = ReferenceTrajectory::target_x - RobotState::current_x;
    float error_y = ReferenceTrajectory::target_y - RobotState::current_y;

    ROS_INFO("Target - x: %f, y: %f, phi: %f", ReferenceTrajectory::target_x, ReferenceTrajectory::target_y, ReferenceTrajectory::target_phi);
    ROS_INFO("Current - x: %f, y: %f, phi: %f", RobotState::current_x, RobotState::current_y, RobotState::current_phi);
    ROS_INFO("Error - x: %f, y: %f", error_x, error_y);

    // Check if within tolerance
    if (std::abs(error_x) < POSITION_TOLERANCE && std::abs(error_y) < POSITION_TOLERANCE ){
        publishMotorCommand(0.0, 0.0);
        ROS_INFO("Target reached: x=%.2f, y=%.2f, phi=%.2f", ReferenceTrajectory::target_x, ReferenceTrajectory::target_y, ReferenceTrajectory::target_phi);
        return;
    }

    // Line Progression Controller: Control forward velocity based on error_x
    float linear_velocity = K_line_progress * error_x;
    
    // Angular velocity based on error_phi
    float angular_velocity[2] = {WHEEL_BASE / WHEEL_RADIUS, -WHEEL_BASE / WHEEL_RADIUS}; // Array for angular velocity
    // Declare left_speed and right_speed outside the conditional blocks
    float left_speed = 0.0;
    float right_speed = 0.0;

    // Determine which controller to use based on the driving state
    if (DrivingState::current_state == "FORWARD") {
        RobotState::w = lineDeviationController(); // Use line deviation controller for FORWARD state
        left_speed = linear_velocity - WHEEL_BASE / WHEEL_RADIUS * RobotState::w;
        right_speed = linear_velocity + WHEEL_BASE / WHEEL_RADIUS * RobotState::w;
    } else if (DrivingState::current_state == "TURNING") {
        RobotState::w = pureRotationController(); // Use pure rotation controller for TURNING state
        left_speed = - WHEEL_BASE / WHEEL_RADIUS * RobotState::w;
        right_speed = WHEEL_BASE / WHEEL_RADIUS * RobotState::w;
    } else if (DrivingState::current_state == "BACKWARD") {
        // potentially set the integral term to zero 
        Error::integral_error_y = 0.0; // Reset integral term for BACKWARD state
        RobotState::w = -lineDeviationController(); // Use line deviation controller for BACKWARD state
        left_speed = -linear_velocity - WHEEL_BASE / WHEEL_RADIUS * RobotState::w;
        right_speed = -linear_velocity + WHEEL_BASE / WHEEL_RADIUS * RobotState::w;
    }

    publishMotorCommand(left_speed, right_speed);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_tracker");
    ros::NodeHandle nh;

    velocity_reference_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/set_reference", 10);
    ros::Subscriber reference_subscriber = nh.subscribe("/reference_trajectory", 10, referenceCallback);
    ros::Subscriber state_subscriber = nh.subscribe("/pose_estimate_fused", 10, stateUpdateCallback);
    driving_state_subscriber = nh.subscribe("/driving_state", 10, drivingStateCallback);

    ros::spin();

    return 0;
}