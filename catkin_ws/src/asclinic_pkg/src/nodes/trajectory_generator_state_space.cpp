#include <ros/ros.h>
#include "asclinic_pkg/referenceVelocityPose.h"
#include "std_msgs/String.h"
// Variables
ros::Publisher motor_reference_position;
ros::Time start_time;

// Helper function to publish position and velocity commands
void publishPositionCommand(float x, float y, float phi, float v, float w) {
    asclinic_pkg::referenceVelocityPose position_command;
    position_command.x = x;
    position_command.y = y;
    position_command.phi = phi;
    position_command.v = v;
    position_command.w = w;
    motor_reference_position.publish(position_command);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_state_space");
    ros::NodeHandle nh;

    motor_reference_position = nh.advertise<asclinic_pkg::referenceVelocityPose>("/reference_trajectory", 10);

    // Pause for 3 seconds at the start
    ros::Duration(3.0).sleep();

    start_time = ros::Time::now();
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
