#include <ros/ros.h>
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/PoseCovar.h"
#include "std_msgs/String.h"

// Constants
const float LINEAR_SPEED = 300;    // in mm/s
//const float Y_SPEED = 50; 
// Variables
ros::Publisher motor_reference_position;
ros::Publisher driving_state_publisher;
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

enum State {
    FORWARD,
    TURNING,
    BACKWARD
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    motor_reference_position = nh.advertise<asclinic_pkg::PoseCovar>("/reference_trajectory", 10);
    driving_state_publisher = nh.advertise<std_msgs::String>("/driving_state", 10);

    // Pause for 5 seconds at the start
    ros::Duration(3.0).sleep();

    start_time = ros::Time::now();
    ros::Rate loop_rate(10); // 10 Hz

    float current_x = 0.0;
    //float current_y = 0.0;
    const float target_x = 5000; // Target position in mm
    State current_state = FORWARD;

    while (ros::ok()) {
        switch (current_state) {
            case FORWARD:
                if (current_x < target_x) {
                    current_x += LINEAR_SPEED / 10.0; // Increment x based on speed and loop rate
                    
                   //current_y  = 0; //+= Y_SPEED / 10.0; // Increment y based on speed and loop rate
                    publishPositionCommand(current_x, 0.0 , 0.0);
                } else {
                    ROS_INFO("Reached 5 meters, transitioning to TURNING state.");
                    current_state = TURNING;
                }
                break;

            case TURNING:
                // Commented out for debugging FORWARD drive
                // ROS_INFO("Initiating 180-degree turn.");
                // publishPositionCommand(current_x, 0.0, 180.0);
                // ros::Duration(5.0).sleep(); // Wait for the turn to complete
                // ROS_INFO("Turn complete, transitioning to BACKWARD state.");
                // current_state = BACKWARD;
                break;

            case BACKWARD:
                // Commented out for debugging FORWARD drive
                // if (current_x > 0.0) {
                //     current_x -= LINEAR_SPEED / 10.0; // Decrement x based on speed and loop rate
                //     publishPositionCommand(current_x, 0.0, 180.0);
                // } else {
                //     ROS_INFO("Returned to the origin, stopping.");
                //     return 0; // Exit the program
                // }
                break;
        }

        // Publish the current driving state
        std_msgs::String driving_state_msg;
        switch (current_state) {
            case FORWARD:
                driving_state_msg.data = "FORWARD";
                break;
            case TURNING:
                driving_state_msg.data = "TURNING";
                break;
            case BACKWARD:
                driving_state_msg.data = "BACKWARD";
                break;
        }
        driving_state_publisher.publish(driving_state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}