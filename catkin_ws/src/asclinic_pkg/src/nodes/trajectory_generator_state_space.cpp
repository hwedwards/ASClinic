#include <ros/ros.h>
#include "asclinic_pkg/referenceVelocityPose.h"
#include "std_msgs/String.h"
#include <fstream>
#include <sstream>
#include <vector>

// Variables
ros::Publisher motor_reference_position;
ros::Publisher driving_state_publisher;
ros::Time start_time;

enum TrajectoryType{ DRIVING, TURNING}; 
struct Trajectory {
    TrajectoryType type;
    std::vector<double> coeffx;
    std::vector<double> coeffy;
    double tf; 
    double phi; 

    // Constructor for DRIVING
    Trajectory(const std::vector<double>& cx, const std::vector<double>& cy, double t)
        : type(DRIVING), coeffx(cx), coeffy(cy), tf(t), phi(0.0) {}

    // Constructor for TURNING
    Trajectory(double p, double t)
        : type(TURNING), coeffx(4, 0.0), coeffy(4, 0.0), tf(t), phi(p) {}
};
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
    driving_state_publisher = nh.advertise<std_msgs::String>("/driving_state", 10);
    // Pause for 3 seconds at the start
    ros::Duration(3.0).sleep();
    ros::Rate loop_rate(10); // 10 Hz

    // Read coefficients from CSV
    std::vector<Trajectory> trajectory_info;
    std::ifstream file("trajectory_coeffs.csv");
    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::getline(ss, val, ',');
        int flag = std::stoi(val);
        if (flag == 0) { // DRIVING
            std::vector<double> cx(4), cy(4);
            for (int i = 0; i < 4; ++i) {
                std::getline(ss, val, ',');
                cx[i] = std::stod(val);
            }
            for (int i = 0; i < 4; ++i) {
                std::getline(ss, val, ',');
                cy[i] = std::stod(val);
            }
            std::getline(ss, val, ',');
            double tf = std::stod(val);
            trajectory_info.emplace_back(cx, cy, tf);
        } else if (flag == 1) { // TURNING
            std::getline(ss, val, ',');
            double phi = std::stod(val);
            std::getline(ss, val, ',');
            double tf = std::stod(val);
            trajectory_info.emplace_back(phi, tf);
            // For TURNING, we only need the angle and time
        }
    }

    start_time = ros::Time::now();
    bool sent_final = false;
    double last_x = 0.0, last_y = 0.0, last_phi = 0.0;
    static double last_phi_sent = 0.0;
    double x, y, dx, dy, phi, v, w, t; 
    // initialise the trajectory coefficients
    // counter
    int i = 0; 
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        t = (now - start_time).toSec();
        // have we reached all the trajectories? 
        if (i >= trajectory_info.size() ) {
            sent_final = true;
            // stop where ever the robot is
            publishPositionCommand(last_x, last_y, last_phi, 0.0, 0.0);
        } else {
            // Are we DRIVING or TURNING?
           if (trajectory_info[i].type == DRIVING){
                // Have we run out of time?
                if (t <= trajectory_info[i].tf) {
                    x = evalCubic(trajectory_info[i].coeffx, t);
                    y = evalCubic(trajectory_info[i].coeffy, t);
                    dx = evalCubicDot(trajectory_info[i].coeffx, t);
                    dy = evalCubicDot(trajectory_info[i].coeffy, t);
                    
                    // I am not currently passing a regerence angle with the Trajectory. FIX THIS!!
                    phi = 0.0;
                    if (dx != 0.0 || dy != 0.0) {
                        phi = std::atan2(dy, dx);
                    }
                    v = std::sqrt(dx*dx + dy*dy);
                    w = (phi - last_phi_sent) / 0.1;
                    last_phi_sent = phi;
                    last_x = x;
                    last_y = y;
                    last_phi = phi;
                    publishPositionCommand(x, y, phi, v, w);
                    sent_final = false;
                } else {
                    // Times up, move to the next trajectory and reset the timer
                    i++; 
                    start_time = ros::Time::now();
                }
            } else {
                // For TURNING, we need to handle the trajectory differently
                if (t <= trajectory_info[i].tf) {
                    // For TURNING, we can use the phi value directly
                    phi = trajectory_info[i].phi;
                    publishPositionCommand(last_x, last_y, phi, v, w);
                } else {
                    i++; 
                    start_time = ros::Time::now();
                }
            }

        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
