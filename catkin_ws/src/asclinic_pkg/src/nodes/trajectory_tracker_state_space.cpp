#include <ros/ros.h>
#include "asclinic_pkg/PoseCovar.h"
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/referenceVelocityPose.h"
#include <cmath>
#include "std_msgs/String.h"
#include <fstream>
#include <sstream>
#include <vector>
// Constants
const float POSITION_TOLERANCE = 0.1; // in meters
const float ANGLE_TOLERANCE = 5.0;    // in degrees
const float WHEEL_RADIUS = 0.072;     // in meters
const float WHEEL_BASE = 0.215 / 2;   // in meters
const float GEAR_RATIO = 70.0;        // Gear ratio
const int RADS_TO_RPM = 9.549;        // conversion factor
const float K_angular = 0.8;          // Proportional gain for angular velocity control
const float Kd_angular = 0;           // Derivative gain for angular velocity control
namespace Gains
{
    std::vector<std::vector<std::vector<float>>> K_x; // [segment][row][col]
    std::vector<std::vector<std::vector<float>>> K_p; // [segment][row][col]
}
ros::Publisher velocity_reference_publisher;
ros::Subscriber driving_state_subscriber;
namespace DrivingState
{
    std::string current_state = "FORWARD"; // Default state
}
void drivingStateCallback(const std_msgs::String &msg)
{
    DrivingState::current_state = msg.data;
}

namespace RobotState
{
    float current_x = 0.0;
    float current_y = 0.0;
    float current_phi = 0.0;
}
namespace ReferenceTrajectory
{
    float target_x = 0.0;
    float target_y = 0.0;
    float target_phi = 0.0;
    float target_v = 0.0; // Target linear velocity
    float target_w = 0.0; // Target angular velocity
    int line_segment_no = 0; // Line segment number for the trajectory
}
namespace Error
{
    float error_x = 0.0;
    float error_y = 0.0;
    float error_phi = 0.0;
    float integral_error_y = 0.0;      // Integral of the lateral error
    float integral_error_x = 0.0;      // Integral of the longitudinal error
    const float INTEGRAL_MAX = 100.0;  // Maximum limit for integral term
    const float INTEGRAL_MIN = -100.0; // Minimum limit for integral term
}
// Function to publish motor commands
void publishMotorCommand(float left_speed, float right_speed)
{
    // Turn radians/s of the wheel into RPM of the motor shaft
    left_speed = left_speed * GEAR_RATIO;
    right_speed = right_speed * GEAR_RATIO;
    // Convert to RPM
    asclinic_pkg::LeftRightFloat32 motor_command;
    motor_command.left = left_speed * RADS_TO_RPM;
    motor_command.right = right_speed * RADS_TO_RPM;
    // ROS_INFO("Publishing motor command - Left: %f, Right: %f", motor_command.left, motor_command.right);
    velocity_reference_publisher.publish(motor_command);
}
// Function to calculate wheel speeds from v and w and publish the command
void calculateAndPublishWheelSpeeds(float v, float w)
{
    // Differential drive kinematics
    float left_speed = (v - WHEEL_BASE * w) / WHEEL_RADIUS;
    float right_speed = (v + WHEEL_BASE * w) / WHEEL_RADIUS;
    publishMotorCommand(left_speed, right_speed);
}
// Callback for reference trajectory
void referenceCallback(const asclinic_pkg::referenceVelocityPose &msg)
{
    // ROS_INFO("Received reference trajectory - x: %f, y: %f, phi: %f, v: %f, w: %f, line_no: %d",
    //          msg.x, msg.y, msg.phi, msg.v, msg.w, msg.line_segment_no);
    
    ReferenceTrajectory::target_x = msg.x / 1000.0f; // Convert to meters
    ReferenceTrajectory::target_y = msg.y / 1000.0f;
    ReferenceTrajectory::target_phi = msg.phi * M_PI/180; // Convert to radians not sure what this is in
    ReferenceTrajectory::target_v = msg.v / 1000.0f;           // Convert mm/s to m/s
    ReferenceTrajectory::target_w = msg.w;     // Already in rad/s not sure about this either
    if (ReferenceTrajectory::line_segment_no != msg.line_segment_no){
        // New segment, reset integral errors
        Error::integral_error_x = 0.0f;
        Error::integral_error_y = 0.0f;
    }
    ReferenceTrajectory::line_segment_no = msg.line_segment_no;

}
double signedAngleDiffDeg(double ref_deg, double meas_deg)
{
    double diff = ref_deg - meas_deg;
    diff = std::fmod(diff + M_PI, 2.0 * M_PI);
    if (diff < 0.0)
        diff += 2.0 * M_PI;
    return diff - M_PI;
}
void pureRotationController(float *w)
{
    float error_phi = signedAngleDiffDeg(ReferenceTrajectory::target_phi, RobotState::current_phi);
    float dphi = (error_phi - Error::error_phi) / 0.1;
    Error::error_phi = error_phi;
    *w = (error_phi * K_angular + Kd_angular * dphi);
}
// LQR-based line following controller
void lineFollowingControllerLQR(float *v, float *w)
{
    int seg = ReferenceTrajectory::line_segment_no;
    if (seg < 0 || seg >= Gains::K_x.size()) seg = 0; // fallback
    float del_v = 0.0f;
    float del_w = 0.0f;
    float v_int = 0.0f;
    float w_int = 0.0f;
    // Full-state feedback control
    del_v = Gains::K_x[seg][0][0] * Error::error_x + Gains::K_x[seg][0][1] * Error::error_y + Gains::K_x[seg][0][2] * Error::error_phi;
    del_w = Gains::K_x[seg][1][0] * Error::error_x + Gains::K_x[seg][1][1] * Error::error_y + Gains::K_x[seg][1][2] * Error::error_phi;
    // Augmented integrator component
    v_int = Gains::K_p[seg][0][0] * Error::integral_error_x + Gains::K_p[seg][0][1] * Error::integral_error_y;
    w_int = Gains::K_p[seg][1][0] * Error::integral_error_x + Gains::K_p[seg][1][1] * Error::integral_error_y;
    // ROS_INFO("Segment %d: del_v: %f, del_w: %f, v_int: %f, w_int: %f",
    //          seg, del_v, del_w, v_int, w_int);
    // ROS_INFO("Errors - x: %f, y: %f, phi: %f, integraler_x: %f, integral_y: %f",
    //          Error::error_x, Error::error_y, Error::error_phi,
    //          Error::integral_error_x, Error::integral_error_y);
    // ROS_INFO("Gains - K_x: [%f, %f, %f], [%f, %f, %f], K_p: [%f, %f], [%f, %f]",
    //          Gains::K_x[seg][0][0], Gains::K_x[seg][0][1], Gains::K_x[seg][0][2],
    //          Gains::K_x[seg][1][0], Gains::K_x[seg][1][1], Gains::K_x[seg][1][2],
    //          Gains::K_p[seg][0][0], Gains::K_p[seg][0][1],
    //          Gains::K_p[seg][1][0], Gains::K_p[seg][1][1]);
    *v = ReferenceTrajectory::target_v + del_v + v_int;
    *w = ReferenceTrajectory::target_w + del_w + w_int;
    // ROS_INFO("[%.3f] v: %f, w: %f (segment %d)", ros::Time::now().toSec(), *v, *w, seg);
}
// Callback to update the robot's current state
void stateUpdateCallback(const asclinic_pkg::PoseCovar &msg)
{
    RobotState::current_x = msg.x / 1000.0f; // Convert to meters from mm
    RobotState::current_y = msg.y / 1000.0f;
    RobotState::current_phi = msg.phi * M_PI / 180.0f; // Convert to radians
    // Log control system information.
    ROS_INFO("[TRACKER] [%.3f] Robot state - x: %f, y: %f, phi: %f",
             ros::Time::now().toSec(),
             RobotState::current_x, RobotState::current_y, RobotState::current_phi);
    ROS_INFO("[TRACKER] [%.3f] Reference trajectory - x: %f, y: %f, phi: %f, v: %f, w: %f, line_no: %d",
             ros::Time::now().toSec(),
             ReferenceTrajectory::target_x, ReferenceTrajectory::target_y,
             ReferenceTrajectory::target_phi, ReferenceTrajectory::target_v,
             ReferenceTrajectory::target_w, ReferenceTrajectory::line_segment_no);
    Error::error_x = ReferenceTrajectory::target_x - RobotState::current_x;
    Error::error_y = ReferenceTrajectory::target_y - RobotState::current_y;
    Error::error_phi = signedAngleDiffDeg(ReferenceTrajectory::target_phi, RobotState::current_phi);
    Error::integral_error_x += Error::error_x * 0.1; // Integral term for longitudinal error
    Error::integral_error_y += Error::error_y * 0.1; // Integral term for lateral error
    // ROS_INFO("[%.3f] Errors - x: %f, y: %f, phi: %f, integral_x: %f, integral_y: %f",
    //          ros::Time::now().toSec(),
    //          Error::error_x, Error::error_y, Error::error_phi,
    //          Error::integral_error_x, Error::integral_error_y);
    // Clamp integral error to prevent windup
    if (Error::integral_error_x > Error::INTEGRAL_MAX)
    {
        Error::integral_error_x = Error::INTEGRAL_MAX;
    }
    else if (Error::integral_error_x < Error::INTEGRAL_MIN)
    {
        Error::integral_error_x = Error::INTEGRAL_MIN;
    }
    if (Error::integral_error_y > Error::INTEGRAL_MAX)
    {
        Error::integral_error_y = Error::INTEGRAL_MAX;
    }
    else if (Error::integral_error_y < Error::INTEGRAL_MIN)
    {
        Error::integral_error_y = Error::INTEGRAL_MIN;
    }

    // Calculate the desired linear and angular velocities
    float v;
    float w;
    if (DrivingState::current_state == "DRIVING")
    {
        // Use the line following controller
        lineFollowingControllerLQR(&v, &w);
    }
    else if (DrivingState::current_state == "TURNING")
    {
        pureRotationController(&w);
        v = 0.0; // No linear velocity during turning
    }
    else
    {
        // Default to zero velocities if not in a valid state
        v = 0.0;
        w = 0.0;
    }

    calculateAndPublishWheelSpeeds(v, w);
}

int main(int argc, char **argv)
{
    // Read all K_p and K_x from trajectoryGainsK.csv
    std::ifstream kfile("/home/asc/ASClinic/trajectoryGainsK.csv");
    std::string kline;
    std::getline(kfile, kline); // skip header
    while (std::getline(kfile, kline)) {
        std::stringstream ss(kline);
        std::string val;
        std::vector<float> K_vals;
        std::getline(ss, val, ','); // segment index
        while (std::getline(ss, val, ',')) {
            K_vals.push_back(std::stof(val));
        }
        // K_x = K(:, 0:2), K_p = K(:, 3:4)
        std::vector<std::vector<float>> Kx(2, std::vector<float>(3, 0.0f));
        std::vector<std::vector<float>> Kp(2, std::vector<float>(2, 0.0f));
        Kx[0][0] = K_vals[0]; Kx[0][1] = K_vals[1]; Kx[0][2] = K_vals[2];
        Kx[1][0] = K_vals[5]; Kx[1][1] = K_vals[6]; Kx[1][2] = K_vals[7];
        Kp[0][0] = K_vals[3]; Kp[0][1] = K_vals[4];
        Kp[1][0] = K_vals[8]; Kp[1][1] = K_vals[9];
        Gains::K_x.push_back(Kx);
        Gains::K_p.push_back(Kp);
    }
    ROS_INFO("Loaded %zu segments of gains", Gains::K_x.size());
    // Print all K_x and K_p
    for (size_t seg = 0; seg < Gains::K_x.size(); ++seg) {
        ROS_INFO("Segment %zu:", seg);
        ROS_INFO("  K_x:");
        for (size_t row = 0; row < Gains::K_x[seg].size(); ++row) {
            std::ostringstream oss;
            oss << "    [";
            for (size_t col = 0; col < Gains::K_x[seg][row].size(); ++col) {
                oss << Gains::K_x[seg][row][col];
                if (col + 1 < Gains::K_x[seg][row].size()) oss << ", ";
            }
            oss << "]";
            ROS_INFO("%s", oss.str().c_str());
        }
        ROS_INFO("  K_p:");
        for (size_t row = 0; row < Gains::K_p[seg].size(); ++row) {
            std::ostringstream oss;
            oss << "    [";
            for (size_t col = 0; col < Gains::K_p[seg][row].size(); ++col) {
                oss << Gains::K_p[seg][row][col];
                if (col + 1 < Gains::K_p[seg][row].size()) oss << ", ";
            }
            oss << "]";
            ROS_INFO("%s", oss.str().c_str());
        }
    }

    ros::init(argc, argv, "trajectory_tracker_state_space");
    ros::NodeHandle nh;
    velocity_reference_publisher = nh.advertise<asclinic_pkg::LeftRightFloat32>("/set_wheel_velocity_reference", 10);
    ros::Subscriber reference_subscriber = nh.subscribe("/reference_trajectory", 10, referenceCallback);
    ros::Subscriber state_subscriber = nh.subscribe("/pose_estimate_fused", 10, stateUpdateCallback);
    driving_state_subscriber = nh.subscribe("/driving_state", 10, drivingStateCallback);
    ros::spin();
    return 0;
}
