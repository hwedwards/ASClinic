// Include necessary libraries
#include <ros/ros.h>
#include <math.h>
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/LeftRightInt32.h"
#include "std_msgs/Float32.h"

// Controller parameters
float Kp = 0.0057; // Proportional gain for velocity control
float Ki = 0.0062; // Integrator gain for velocity control
float reference = 0.0; // Desired reference value
float integrator = 0.0; // Integrator state
float previous_error = 0.0; // Previous error for integration

// Encoder frequency assumption
const float ENCODER_FREQUENCY_HZ = 50.0; // Encoder frequency in Hz
const float DELTA_T = 1.0 / ENCODER_FREQUENCY_HZ; // Time step in seconds

// Publisher for motor duty cycle
ros::Publisher motor_duty_cycle_publisher;

// Callback for encoder counts
void encoderCountsCallback(const asclinic_pkg::LeftRightInt32& msg)
{
    // Compute the current state (e.g., velocity or position)
    float current_state = static_cast<float>(msg.left + msg.right) / 2.0;

    // Compute the error
    float error = reference - current_state;

    // Update the integrator
    integrator += error * DELTA_T;

    // Compute the control action
    float control_action = Kp * error + Ki * integrator;

    // Prepare and publish the motor duty cycle message
    asclinic_pkg::LeftRightFloat32 duty_cycle_msg;
    duty_cycle_msg.left = control_action;
    duty_cycle_msg.right = control_action;
    motor_duty_cycle_publisher.publish(duty_cycle_msg);
}

// Callback for reference value
void referenceCallback(const std_msgs::Float32& msg)
{
    reference = msg.data;
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