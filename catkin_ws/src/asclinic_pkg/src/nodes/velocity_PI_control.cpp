#include <ros/ros.h>
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/LeftRightInt32.h"

// to run this node, rostopic pub --once /set_reference asclinic_pkg/LeftRightFloat32 "{left: 2000.0, right: 2000.0}"
namespace ControllerParameters
{
    // Controller parameters
    float Kp_left = 0.001; // Proportional gain for velocity control
    float Ki_left = 0.001; // Integrator gain for velocity control
    float Kp_right = 0.001; 
    float Ki_right = 0.001; // Integrator gain for velocity control
    float reference_left = 0.0; // Desired reference value
    float reference_right = 0.0; // Desired reference value
    float integrator_left = 0.0; // Integrator state
    float integrator_right = 0.0; // Integrator state
    float previous_error = 0.0; // Previous error for integration
}

// Encoder frequency assumption
const float ENCODER_FREQUENCY_HZ = 10.0; // Encoder frequency in Hz
const float DELTA_T = 1.0 / ENCODER_FREQUENCY_HZ; // Time step in seconds
const float ENCODER_COUNTS_PER_REVOLUTION_LEFT = 16.0; 
const float ENCODER_COUNTS_PER_REVOLUTION_RIGHT = 16.0; // Encoder counts per revolution

//Motor Constants
const float MOTOR_MAX_VOLTAGE = 12.0; // Maximum voltage of the motor
// Publisher for motor duty cycle
ros::Publisher motor_duty_cycle_publisher;

// Callback for encoder counts
void encoderCountsCallback(const asclinic_pkg::LeftRightInt32& msg)
{
    // Log received encoder counts
    ROS_INFO("Received encoder counts - Left: %d, Right: %d", msg.right, msg.left);

    // NOTE: the left and right encoder counts are swapped in the message!!!!
    // Compute the current state velocity in terms of RPM (How the controller was designed)
    float current_state_left = (static_cast<float>(msg.right) * ENCODER_FREQUENCY_HZ*60) / ENCODER_COUNTS_PER_REVOLUTION_LEFT;
    float current_state_right = (static_cast<float>(msg.left) * ENCODER_FREQUENCY_HZ*60) / ENCODER_COUNTS_PER_REVOLUTION_RIGHT;
    
    ROS_INFO("current state - Left: %f, Right: %f", current_state_left, current_state_right);
    // Compute the error
    float error_left = ControllerParameters::reference_left - current_state_left; 
    float error_right = ControllerParameters::reference_right - current_state_right;
    ROS_INFO("error_term - Left: %f, Right: %f", error_left, error_right);

    // the state feedback is multiplying the FUCKING ERROR!!!!!!
    float state_feedback_control_left = ControllerParameters::Kp_left * error_left;
    float state_feedback_control_right = ControllerParameters::Kp_right * error_right;
    
    ROS_INFO("state feedback term - Left: %f, Right: %f", state_feedback_control_left, state_feedback_control_right);
    
    ControllerParameters::integrator_left += error_left * DELTA_T;
    ControllerParameters::integrator_right += error_right * DELTA_T;
    
    float integrator_control_left = ControllerParameters::integrator_left * ControllerParameters::Ki_left;
    float integrator_control_right = ControllerParameters::integrator_right * ControllerParameters::Ki_right;

    ROS_INFO("integrator term - Left: %f, Right: %f", integrator_control_left, integrator_control_right);
    //  // It's definitely the integrator term thats screwing things up a bit
    
    // Compute the control action
    float control_action_left = state_feedback_control_left+ integrator_control_left;
    float control_action_right = state_feedback_control_right + integrator_control_right;

    
    // I need to convert the control action into a duty cycle

    control_action_left = control_action_left*100/MOTOR_MAX_VOLTAGE; 

    control_action_right = control_action_right*100/MOTOR_MAX_VOLTAGE;

    // Prepare and publish the motor duty cycle message
    asclinic_pkg::LeftRightFloat32 duty_cycle_msg;
    duty_cycle_msg.left = control_action_left;
    duty_cycle_msg.right = control_action_right;
    motor_duty_cycle_publisher.publish(duty_cycle_msg);

    // Log computed duty cycles
    ROS_INFO("Computed duty cycles - Left: %f, Right: %f", duty_cycle_msg.left, duty_cycle_msg.right);
}

// Callback for reference values for both wheels
void referenceCallback(const asclinic_pkg::LeftRightFloat32& msg)
{
    // Log received reference values
    ROS_INFO("Received reference velocities - Left: %f RPM, Right: %f RPM", msg.left, msg.right);

    ControllerParameters::reference_left = msg.left;
    ControllerParameters::reference_right = msg.right;
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "velocity_PI_control_node");
    ros::NodeHandle nh;

    // Initialize the publisher
    motor_duty_cycle_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);

    // Initialize the subscribers
    ros::Subscriber encoder_subscriber = nh.subscribe("/asc/encoder_counts", 10, encoderCountsCallback);
    ros::Subscriber reference_subscriber = nh.subscribe("set_reference", 10, referenceCallback);

    // Spin the node
    ros::spin();

    return 0;
}