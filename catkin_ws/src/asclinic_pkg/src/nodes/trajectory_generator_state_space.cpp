#include <ros/ros.h>
#include "asclinic_pkg/referenceVelocityPose.h"
#include "std_msgs/String.h"
#include <fstream>
#include <sstream>
#include <vector>

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

// Helper function to evaluate cubic polynomial and its derivative
double evalCubic(const std::vector<double>& coeffs, double t) {
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t;
}
double evalCubicDot(const std::vector<double>& coeffs, double t) {
    return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t*t;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_state_space");
    ros::NodeHandle nh;

    motor_reference_position = nh.advertise<asclinic_pkg::referenceVelocityPose>("/reference_trajectory", 10);

    // Pause for 3 seconds at the start
    ros::Duration(3.0).sleep();
    ros::Rate loop_rate(10); // 10 Hz

    // Read coefficients from CSV
    std::vector<double> coeffx(4, 0.0), coeffy(4, 0.0);
    std::ifstream file("/home/asc/ASClinic/trajectory_coeffs.csv");
    std::string line;
    if (file.good()) {
        std::getline(file, line); // skip header
        if (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string val;
            for (int i = 0; i < 4; ++i) {
                std::getline(ss, val, ',');
                coeffx[i] = std::stod(val);
            }
            for (int i = 0; i < 4; ++i) {
                std::getline(ss, val, ',');
                coeffy[i] = std::stod(val);
            }
        }
    } else {
        ROS_ERROR("Could not open trajectory_coeffs.csv");
        return 1;
    }

    start_time = ros::Time::now();
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        double t = (now - start_time).toSec();
        // Calculate x(t), y(t), phi(t), v(t), w(t)
        double x = evalCubic(coeffx, t);
        double y = evalCubic(coeffy, t);
        double dx = evalCubicDot(coeffx, t);
        double dy = evalCubicDot(coeffy, t);
        double phi = 0.0;
        double v = std::sqrt(dx*dx + dy*dy); // linear velocity
        double w = 0.0;
        publishPositionCommand(x, y, phi, v, w);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
