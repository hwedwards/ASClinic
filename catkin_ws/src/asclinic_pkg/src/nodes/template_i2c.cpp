// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
// This file is part of ASClinic-System.
//    
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// Template node for opening an I2C device
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"

#include "i2c_driver/i2c_driver.h"


// Respond to subscriber receiving a message
void templateSubscriberCallback(const std_msgs::Int32& msg)
{
	ROS_INFO_STREAM("[TEMPLATE GPIO] Message receieved with data = " << msg.data);
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "template_i2c");
	ros::NodeHandle nodeHandle("~");
	// Initialise a publisher
	ros::Publisher gpio_event_publisher = nodeHandle.advertise<std_msgs::Int32>("i2c_data", 10, false);
	// Initialise a subscriber
	ros::Subscriber gpio_event_subscriber = nodeHandle.subscribe("i2c_data", 1, templateSubscriberCallback);


	// Specify the name of the I2C interface
	const char * i2c_device_name = "/dev/i2c-1";

	// Instantiate an "i2c_driver" object
	I2C_Driver i2c_driver (i2c_device_name);


	// Open the I2C device
	bool openSuccess = i2c_driver.open_i2c_device();

	// Display the status
	if (!openSuccess)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C] FAILED to open I2C device named " << i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[TEMPLATE I2C] Successfully opened named " << i2c_driver.get_device_name() << ", with file descriptor = " << i2c_driver.get_file_descriptor());
	}

	// Enter a loop that continues while ROS is still running
	while (ros::ok())
	{
		// Spin once so that this node can service any
		// callbacks that this node has queued.
		ros::spinOnce();

	}

	// Close the I2C device
	bool closeSuccess = i2c_driver.close_i2c_device();
	
	// Display the status
	if (!closeSuccess)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C] FAILED to close I2C device named " << i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[TEMPLATE I2C] Successfully closed device named " << i2c_driver.get_device_name());
	}

	return 0;
}
