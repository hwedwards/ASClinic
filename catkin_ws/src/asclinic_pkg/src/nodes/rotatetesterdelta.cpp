#include "ros/ros.h"
#include <ros/package.h>
#include <math.h>
#include "asclinic_pkg/PoseCovar.h"
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/LeftRightFloat32.h"

#define THRESHOLD_DISTANCE 3000 // in mm
#define THRESHOLD_TICKS 5000    // 1620   // in ticks
#define SPEED 20                // in % for PWM duty cycle

// use global variables, static means scope is limited to this script
static float d = 0, pose_x = 0, pose_y = 0, pose_phi = 0;
static float pose_xvar = 0, pose_yvar = 0, pose_phivar = 0, pose_xycovar = 0, pose_xphicovar = 0, pose_yphicovar = 0;
static int leftcount = 0, rightcount = 0, seq_k = 0;

// whenever a message comes to the '/asc/encoder_counts' topic, this callback is executed
void calculate_distance(const asclinic_pkg::PoseCovar &msg)
{
    // Calculate the distance based on the change in position
    d += sqrt(pow(msg.x, 2) + pow(msg.y, 2));
    pose_x += msg.x;       // no longer x coordinate of pose but rather total x-displacements
    pose_y += msg.y;       // total y-displacements
    pose_phi += msg.phi;   // total phi-displacements
    pose_xvar += msg.xvar; // total variances
    pose_yvar += msg.yvar;
    pose_phivar += msg.phivar;
    pose_xycovar += msg.xycovar; // total covariances
    pose_xphicovar += msg.xphicovar;
    pose_yphicovar += msg.yphicovar;
}

void countticks(const asclinic_pkg::LeftRightInt32 &msg)
{
    leftcount += msg.right; // if the right wheel turns, the left encoder count increases, and vice versa
    rightcount += msg.left;
}

int main(int argc, char *argv[])
{
    d = 0;
    pose_x = 0;
    pose_y = 0;
    leftcount = 0;
    rightcount = 0;
    pose_phi = 0;

    // Initialise the node
    ros::init(argc, argv, "rotatetester");

    // Initialise a node handle to the group namespace
    std::string ns_for_group = ros::this_node::getNamespace();
    ros::NodeHandle nh_for_group(ns_for_group);

    ros::Rate loop_rate(10); // encoder_counts published at fs = 10 Hz

    // Subscribe to /Pose
    ros::Subscriber pose_subscriber = nh_for_group.subscribe("Pose", 10, calculate_distance);
    // Subscribe to /asc/encoder_counts
    ros::Subscriber encodersubscriber = nh_for_group.subscribe("/asc/encoder_counts", 1, countticks);

    // Initialise a publisher
    ROS_INFO("Initial pose at: (%f, %f, phi: %f)", pose_x, pose_y, pose_phi);
    ros::Publisher m_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("/asc/set_motor_duty_cycle", 10);
    asclinic_pkg::LeftRightFloat32 dutycycle;
    /*
        ros::Duration(1).sleep();
        dutycycle.left = SPEED;
        dutycycle.right = SPEED;
        dutycycle.seq_num = 0;

        m_publisher.publish(dutycycle);
        ROS_INFO("Duty cycle = %.2f, %.2f, Distance travelled = %.2f", dutycycle.left, dutycycle.right, d);
        ROS_INFO("To begin, ticks counted left: %d, right: %d", leftcount, rightcount);
        // Alternatively if you want to run it for a certain time, get rid of the while loop and do this:
        // Wait 1 seconds
        // ros::Duration(1).sleep();
        // dutycycle.left = 0;
        // dutycycle.right = 0;

        // Then publish final message

        // m_publisher.publish(dutycycle);
        // ROS_INFO("Message = %f, %f, %d", dutycycle.left, dutycycle.right, dutycycle.seq_num);

        // Spin at a specific rate, 10Hz here
        while (ros::ok())
        {
            ros::spinOnce();

            ROS_INFO("Node is running, Distance travelled: %.2f, L: %d, R: %d", d, leftcount, rightcount);

            // if (leftcount > THRESHOLD_TICKS)
            if (rightcount > THRESHOLD_TICKS)
            {
                dutycycle.left = 0;
                dutycycle.right = 0;
                m_publisher.publish(dutycycle);
                ros::Duration(1).sleep();
                pose_phi = final_phi - pose_phi;
                ros::Duration(0).sleep();
                ROS_INFO("Duty cycle = %.2f, %.2f, Distance travelled = %.2f", dutycycle.left, dutycycle.right, d);
                ROS_INFO("Final pose at: (%f, %f, phi: %f)", pose_x, pose_y, pose_phi);
                ROS_INFO("Total ticks counted left: %d, right: %d", leftcount, rightcount);
                break;
            }
            loop_rate.sleep();
        }
        */
    ROS_INFO("Phase 1: Robot moves itself...");

    // Phase 1: Move with motors after 1 second
    ros::Duration(1).sleep();
    dutycycle.left = SPEED + 0.85; // + 0.85 makes path of robot travel more straight
    dutycycle.right = SPEED;
    dutycycle.seq_num = 0;
    m_publisher.publish(dutycycle);

    while (ros::ok() && d < THRESHOLD_DISTANCE)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop motors after auto move
    dutycycle.left = 0;
    dutycycle.right = 0;
    m_publisher.publish(dutycycle);

    ROS_INFO("Phase 2: Move robot manually for 5 seconds...");

    // Phase 2: Passive listening
    ros::Time manual_start = ros::Time::now();
    while (ros::ok() && ros::Time::now() - manual_start < ros::Duration(5.0))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("=== Movement Summary ===");
    ROS_INFO("Total distance travelled: %.2f mm", d);
    ROS_INFO("Final displacements: (%.2f, %.2f), phi: %.2f", pose_x, pose_y, pose_phi);
    ROS_INFO("Final variances: (%.2f, %.2f, %.2f)", pose_xvar, pose_yvar, pose_phivar);
    ROS_INFO("Final covariances: (%.2f, %.2f, %.2f)", pose_xycovar, pose_xphicovar, pose_yphicovar);
    ROS_INFO("Total encoder ticks: Left = %d, Right = %d", leftcount, rightcount);

    return 0;
}