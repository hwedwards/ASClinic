#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <vector>

ros::Publisher obstacle_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int total_ranges = scan->ranges.size();
    int front_samples = 10; // 10 samples from each side of the front
    std::vector<float> front_angles;
    int center = total_ranges / 2; // find the rear centre index

    // Combine front-right and front-left scan values
    for (int i = -front_samples; i <= front_samples; ++i)
    {
        int idx = center + i;
        if (idx >= 0 && idx < total_ranges)
        {
            front_angles.push_back(scan->ranges[idx]);
        }
    }

    // Find the minimum distance
    float min_distance = *std::min_element(front_angles.begin(), front_angles.end());

    std_msgs::Bool msg;
    if (min_distance < 0.5)
    {
        // ROS_WARN("Obstacle detected at %.2f meters!", min_distance);
        msg.data = true;
    }
    else
    {
        // ROS_INFO("Path is clear.");
        msg.data = false;
    }

    obstacle_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;

    // Publisher for obstacle detection status
    obstacle_pub = nh.advertise<std_msgs::Bool>("/obstacle_detected", 10);

    // Subscriber for laser scan data
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    ros::spin();
    return 0;
}