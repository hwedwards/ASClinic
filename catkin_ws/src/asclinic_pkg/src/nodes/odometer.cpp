#include "ros/ros.h"
#include <ros/package.h>
#include <math.h>
#include "asclinic_pkg/PoseCovar.h"
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/LeftRightFloat32.h"

#define WHEELRADIUS 72
#define WHEELBASETWO 215 // 2b = 215
#define COUNTS_PER_REV 16
#define LEFTCOUNTSPERREV 1123.1
#define RIGHTCOUNTSPERREV 1129.4
#define KL 0.0003862 // proportionality constant between total angular displacement and variance of angular displacement for the left wheel
#define KR 0.0006456 // same for the right wheel

// use global variables, static means scope is limited to this script
static int left_encoder_count = 0, right_encoder_count = 0; // Number of ticks counted in the last time step
static float delta_theta_l = 0, delta_theta_r = 0, delta_s = 0, delta_phi = 0;
static float theta_dot_l = 0, theta_dot_r = 0;

static int dir_l = 1; // track the direction of the left wheel, positive if forwards
static int dir_r = 1; // track the direction of the right wheel, negative if backwards

static float a, b, c, d, e, f, ssin, scos, rcos, rsin, rbssin, rbscos, j, k, l, m; // intermediate variables in pose covariance calculation
static float lvar = 0, rvar = 0;

// whenever a message comes to the '/asc/encoder_counts' topic, this callback is executed
void setencodercounts(const asclinic_pkg::LeftRightInt32 &msg)
{
    left_encoder_count = msg.right; // if the right wheel turns, the left encoder count increases, and vice versa
    right_encoder_count = msg.left;
    // ROS_INFO_STREAM("Message received with data: " << left_encoder_count);
}

void setdirection(const asclinic_pkg::LeftRightFloat32 &msg)
{
    if (msg.left >= 0)
    {
        dir_l = 1;
    }
    else
    {
        dir_l = -1;
    }
    if (msg.right >= 0)
    {
        dir_r = 1;
    }
    else
    {
        dir_r = -1;
    }
    // ROS_INFO_STREAM("Message received with data: " << left_encoder_count);
}

int main(int argc, char *argv[])
{
    // Initialise the node
    ros::init(argc, argv, "odometer");

    // Initialise a node handle to the group namespace
    std::string ns_for_group = ros::this_node::getNamespace();
    // ROS_INFO("Namespace: %s", ns.c_str());
    ros::NodeHandle nh_for_group(ns_for_group);

    ros::Rate loop_rate(10); // encoder_counts published at fs = 10 Hz

    // Subscribe to /asc/encoder_counts
    ros::Subscriber encodersubscriber = nh_for_group.subscribe("/asc/encoder_counts", 1, setencodercounts);

    // Subscribe to /asc/set_motor_duty_cycle
    ros::Subscriber dutycyclesubscriber = nh_for_group.subscribe("/asc/set_motor_duty_cycle", 1, setdirection);

    // Initialise a publisher
    ros::Publisher m_publisher = nh_for_group.advertise<asclinic_pkg::PoseCovar>("Pose", 10);

    asclinic_pkg::PoseCovar pose;
    pose.x = 0;
    pose.y = 0;
    pose.phi = 0;
    pose.xvar = 0;
    pose.yvar = 0;
    pose.phivar = 0;
    pose.xycovar = 0;
    pose.xphicovar = 0;
    pose.yphicovar = 0;
    m_publisher.publish(pose);
    // ROS_INFO("Message = %d", msg);

    // Spin as a single-threaded node
    // ros::Rate loop_rate(10);

    // Spin at a specific rate, 2Hz here
    while (ros::ok())
    {
        // ROS_INFO("Node is running, message is %d", msg.data);
        delta_theta_l = 2 * dir_l * M_PI * left_encoder_count / LEFTCOUNTSPERREV;
        delta_theta_r = 2 * dir_r * M_PI * right_encoder_count / RIGHTCOUNTSPERREV;
        delta_s = (delta_theta_r + delta_theta_l) * WHEELRADIUS / 2;
        delta_phi = (delta_theta_r - delta_theta_l) * WHEELRADIUS / WHEELBASETWO;
        pose.x = pose.x + delta_s * cos(pose.phi + 0.5 * delta_phi);
        pose.y = pose.y + delta_s * sin(pose.phi + 0.5 * delta_phi);
        pose.phi = fmod(pose.phi + delta_phi, 2 * M_PI); // std::fmod() if <cmath> is used instead
        if (pose.phi < 0)
        {
            pose.phi += 2 * M_PI;
        }

        lvar = KL * abs(delta_theta_l);
        rvar = KR * abs(delta_theta_r);

        a = pose.xvar; // covariance matrix values from the previous timestep
        b = pose.yvar;
        c = pose.phivar;
        d = pose.xycovar;
        e = pose.xphicovar;
        f = pose.yphicovar;
        ssin = -delta_s * sin(pose.phi - 0.5 * delta_phi); // pose.phi is from current timestep
        scos = delta_s * cos(pose.phi - 0.5 * delta_phi);  // previous timestep phi plus half the step is the same as current timestep minus half the step
        rcos = WHEELRADIUS / 2 * cos(pose.phi - 0.5 * delta_phi);
        rsin = WHEELRADIUS / 2 * sin(pose.phi - 0.5 * delta_phi);
        rbssin = -WHEELRADIUS / (2 * WHEELBASETWO) * ssin;
        rbscos = WHEELRADIUS / (2 * WHEELBASETWO) * scos;
        j = rcos + rbssin;
        k = rcos + rbssin;
        l = rsin - rbscos;
        m = rsin + rbscos;

        pose.xvar = a + 2 * e * ssin + c * ssin * ssin + lvar * j * j + rvar * k * k;
        pose.yvar = b + 2 * f * scos + c * scos * scos + lvar * l * l + rvar * m * m;
        pose.phivar = c + (lvar + rvar) * WHEELRADIUS / WHEELBASETWO * WHEELRADIUS / WHEELBASETWO;
        pose.xycovar = d + f * ssin + e * scos + c * ssin * scos + lvar * j * l + rvar * k * m;
        pose.xphicovar = e + c * ssin + (-lvar * j + rvar * k) * WHEELRADIUS / WHEELBASETWO;
        pose.yphicovar = f + c * scos + (-lvar * l + rvar * m) * WHEELRADIUS / WHEELBASETWO;

        m_publisher.publish(pose);
        asclinic_pkg::LeftRightFloat32 wheel_velocity_rpm_msg;
        wheel_velocity_rpm_msg.left = delta_theta_l;
        wheel_velocity_rpm_msg.right = delta_theta_r;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}