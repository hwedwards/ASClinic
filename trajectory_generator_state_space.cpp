#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include "your_package/PositionCommand.h" // Adjust the include according to your package

// Function to evaluate cubic polynomial
double evalCubic(const std::vector<double>& coeff, double t) {
    return coeff[0] + coeff[1]*t + coeff[2]*t*t + coeff[3]*t*t*t;
}

// Function to evaluate derivative of cubic polynomial
double evalCubicDot(const std::vector<double>& coeff, double t) {
    return coeff[1] + 2*coeff[2]*t + 3*coeff[3]*t*t;
}

ros::Publisher position_pub;
ros::Time start_time;

void publishPositionCommand(double x, double y, double phi, double v, double w) {
    your_package::PositionCommand cmd;
    cmd.x = x;
    cmd.y = y;
    cmd.phi = phi;
    cmd.v = v;
    cmd.w = w;
    position_pub.publish(cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_follower");
    ros::NodeHandle nh;

    position_pub = nh.advertise<your_package::PositionCommand>("position_command", 10);

    // Read coefficients from CSV
    std::vector<double> coeffx(4, 0.0), coeffy(4, 0.0);
    double tf = 0.0;
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
            // Read tf
            if (std::getline(ss, val, ',')) {
                tf = std::stod(val);
            }
        }
    } else {
        ROS_ERROR("Could not open trajectory_coeffs.csv");
        return 1;
    }

    start_time = ros::Time::now();
    bool sent_final = false;
    double last_x = 0.0, last_y = 0.0, last_phi = 0.0;
    static double last_phi_sent = 0.0;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        double t = (now - start_time).toSec();
        double x, y, dx, dy, phi, v, w;
        if (t <= tf) {
            x = evalCubic(coeffx, t);
            y = evalCubic(coeffy, t);
            dx = evalCubicDot(coeffx, t);
            dy = evalCubicDot(coeffy, t);
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
        } else if (!sent_final) {
            // After tf, hold the last position and orientation, v and w = 0
            publishPositionCommand(last_x, last_y, last_phi, 0.0, 0.0);
            sent_final = true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}