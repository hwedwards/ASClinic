// Include necessary libraries
#include <ros/ros.h>
#include <math.h>
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/LeftRightInt32.h"
#include "std_msgs/Float32.h"
#include <ros/console.h>

namespace ControllerParameters
{
    // Controller parameters
    float Kp = 0.0057; // Proportional gain for velocity control
    float Ki = 0.0062; // Integrator gain for velocity control
    float reference_left = 0.0; // Desired reference value
    float reference_right = 0.0; // Desired reference value
    float integrator_left = 0.0; // Integrator state
    float integrator_right = 0.0; // Integrator state
    float previous_error = 0.0; // Previous error for integration
}

// Encoder frequency assumption
const float ENCODER_FREQUENCY_HZ = 50.0; // Encoder frequency in Hz
const float DELTA_T = 1.0 / ENCODER_FREQUENCY_HZ; // Time step in seconds
const float ENCODER_COUNTS_PER_REVOLUTION_LEFT = 16.12; 
const float ENCODER_COUNTS_PER_REVOLUTION_RIGHT = 16.43; // Encoder counts per revolution

//Motor Constants
const float MOTOR_MAX_VOLTAGE = 12.0; // Maximum voltage of the motor
// Publisher for motor duty cycle
ros::Publisher motor_duty_cycle_publisher;

// Callback for encoder counts
void encoderCountsCallback(const asclinic_pkg::LeftRightInt32& msg)
{
    // Log received encoder counts
    ROS_INFO("Received encoder counts - Left: %d, Right: %d", msg.left, msg.right);

    // Compute the current state (e.g., velocity or position)
    float current_state_left = static_cast<float>(msg.left) * ENCODER_FREQUENCY_HZ / ENCODER_COUNTS_PER_REVOLUTION_LEFT;
    float current_state_right = static_cast<float>(msg.right) * ENCODER_FREQUENCY_HZ / ENCODER_COUNTS_PER_REVOLUTION_RIGHT;

    // Compute the error
    float error_left = ControllerParameters::reference_left - current_state_left; 
    float error_right = ControllerParameters::reference_right - current_state_right;

    // Update the integrator
    ControllerParameters::integrator_left += error_left * DELTA_T;
    ControllerParameters::integrator_right += error_right * DELTA_T;

    // Compute the control action
    float control_action_left = ControllerParameters::Kp * current_state_left + ControllerParameters::Ki * ControllerParameters::integrator_left;
    float control_action_right = ControllerParameters::Kp * current_state_right + ControllerParameters::Ki * ControllerParameters::integrator_right;

    // I need to convert the control action into a duty cycle

    control_action_left = control_action_left*100/MOTOR_MAX_VOLTAGE; 

    control_action_right = control_action_right*100/MOTOR_MAX_VOLTAGE;

    // Prepare and publish the motor duty cycle message
    asclinic_pkg::LeftRightFloat32 duty_cycle_msg;
    duty_cycle_msg.left = control_action_left;
    duty_cycle_msg.right = control_action_right;
    motor_duty_cycle_publisher.publish(duty_cycle_msg);

    // Log computed duty cycles
    ROS_INFO("Computed duty cycles - Left: %.2f, Right: %.2f", duty_cycle_msg.left, duty_cycle_msg.right);
}

// Callback for reference values for both wheels
void referenceCallback(const asclinic_pkg::LeftRightFloat32& msg)
{
    // Log received reference values
    ROS_INFO("Received reference velocities - Left: %.2f RPM, Right: %.2f RPM", msg.left, msg.right);

    ControllerParameters::reference_left = msg.left;
    ControllerParameters::reference_right = msg.right;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    // Initialize the publisher
    motor_duty_cycle_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("set_motor_duty_cycle", 10);

    // Initialize the subscribers
    ros::Subscriber encoder_subscriber = nh.subscribe("encoder_counts", 10, encoderCountsCallback);
    ros::Subscriber reference_subscriber = nh.subscribe("set_reference", 10, referenceCallback);

    // Spin the node
    ros::spin();

    return 0;
}