#include <ros/ros.h>
#include "asclinic_pkg/LeftRightFloat32.h"

// Constants
const float TARGET_DISTANCE = 11.9; // in meters
const float LINEAR_SPEED = 0.5;    // in m/s
const float ANGULAR_SPEED = 30.0;  // in degrees/s

// FSM States
enum State {
    FORWARD,
    TURN,
    BACKWARD,
    STOP
};

// Variables
State current_state = FORWARD;
float current_position = 0.0; // in meters
float current_angle = 0.0;    // in degrees
ros::Publisher motor_duty_cycle_publisher;

// Helper function to publish motor commands
void publishMotorCommand(float left_speed, float right_speed) {
    asclinic_pkg::LeftRightFloat32 duty_cycle_msg;
    duty_cycle_msg.left = left_speed;
    duty_cycle_msg.right = right_speed;
    motor_duty_cycle_publisher.publish(duty_cycle_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    motor_duty_cycle_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        switch (current_state) {
            case FORWARD:
                if (current_position < TARGET_DISTANCE) {
                    publishMotorCommand(LINEAR_SPEED, LINEAR_SPEED);
                    current_position += LINEAR_SPEED / 10.0; // Update position (10 Hz loop)
                } else {
                    current_state = TURN;
                }
                break;

            case TURN:
                if (current_angle < 180.0) {
                    publishMotorCommand(-ANGULAR_SPEED, ANGULAR_SPEED);
                    current_angle += ANGULAR_SPEED / 10.0; // Update angle (10 Hz loop)
                } else {
                    current_state = BACKWARD;
                }
                break;

            case BACKWARD:
                if (current_position > 0.0) {
                    publishMotorCommand(-LINEAR_SPEED, -LINEAR_SPEED);
                    current_position -= LINEAR_SPEED / 10.0; // Update position (10 Hz loop)
                } else {
                    current_state = STOP;
                }
                break;

            case STOP:
                publishMotorCommand(0.0, 0.0);
                ROS_INFO("Trajectory complete.");
                ros::shutdown();
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}