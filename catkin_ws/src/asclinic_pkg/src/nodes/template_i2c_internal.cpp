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
// Template node for I2C devices connected inside the robot
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

#include "i2c_driver/i2c_driver.h"

#include "pololu_smc_g2/pololu_smc_g2.h"

#include "pca9685/pca9685.h"

#include <bitset>

// Include the asclinic message types
#include "asclinic_pkg/ServoPulseWidth.h"

// Namespacing the package
using namespace asclinic_pkg;





// MEMBER VARIABLES FOR THIS NODE:
// > For the I2C driver
const char * m_i2c_device_name = "/dev/i2c-1";
I2C_Driver m_i2c_driver (m_i2c_device_name);

// > For the Pololu Simple Motor Controller (SMC) driver
const uint8_t m_pololu_smc_address_left = 13;
Pololu_SMC_G2 m_pololu_smc_left (&m_i2c_driver, m_pololu_smc_address_left);

const uint8_t m_pololu_smc_address_right = 14;
Pololu_SMC_G2 m_pololu_smc_right (&m_i2c_driver, m_pololu_smc_address_right);

// > For the PCA9685 PWM Servo Driver driver
const uint8_t m_pca9685_address = 0x42;
PCA9685 m_pca9685_servo_driver (&m_i2c_driver, m_pca9685_address);


// > Publisher and timer for the current
//   measurements
ros::Publisher m_current_publisher;
ros::Timer m_timer_for_publishing;



// Respond to subscriber receiving a message
// > To test this out without creating an additional
//   ROS node
//   1) First use the command:
//        rostopic list
//      To identify the full name of this subscription topic.
//   2) Then use the following command to send  message on
//      that topic
//        rostopic pub --once <namespace>/set_motor_duty_cycle std_msgs/UInt16 10
//      where "<namespace>/set_motor_duty_cycle" is the full
//      name identified in step 1.
//
void templateSubscriberCallback(const std_msgs::UInt16& msg)
{
	ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Message receieved with data = " << msg.data);

	// Clip the data to be in the range [0,100]
	int pwm_duty_cycle = msg.data;
	if (pwm_duty_cycle < 0)
		pwm_duty_cycle = 0;
	if (pwm_duty_cycle > 100)
		pwm_duty_cycle = 100;

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	Pololu_SMC_G2 * pololu_smc_pointer;

	// Initialise one boolean variable for the result
	// of all calls to Pololu_SMC_G2 functions
	bool result;

	// Set the target speed to be the same for
	// both motor controllers

	// Iterate over the pololu objects
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		// Point to the appropriate motor controller
		if (i_smc==0)
			pololu_smc_pointer = &m_pololu_smc_left;
		else
			pololu_smc_pointer = &m_pololu_smc_right;

		// Set the target speed
		result = pololu_smc_pointer->set_motor_target_speed_percent(pwm_duty_cycle);
		if (!result)
			ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
	}


	// Get the target speed value to check that
	// it was set correctly

	// Iterate over the pololu objects
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		// Point to the appropriate motor controller
		if (i_smc==0)
			pololu_smc_pointer = &m_pololu_smc_left;
		else
			pololu_smc_pointer = &m_pololu_smc_right;

		int16_t current_target_speed_value;
		result = pololu_smc_pointer->get_target_speed_3200(&current_target_speed_value);
		if (result)
		{
			ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Pololu SMC - get target speed value returned: " << current_target_speed_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED - Pololu SMC - get target speed value NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
		}
	}
}


void templateServoSubscriberCallback(const ServoPulseWidth& msg)
{
	// Extract the channel and pulse width from the message
	uint8_t channel = msg.channel;
	uint16_t pulse_width_in_us = msg.pulse_width_in_microseconds;

	// Display the message received
	ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Message receieved for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

	// Limit the pulse width to be either:
	// > zero
	// > in the range [1000,2000]
	if (pulse_width_in_us > 0)
	{
		if (pulse_width_in_us < 1000)
			pulse_width_in_us = 1000;
		if (pulse_width_in_us > 2000)
			pulse_width_in_us = 2000;
	}

	// Call the function to set the desired pulse width
	bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

	// Display if an error occurred
	if (!result)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
	}

}

// Respond to timer callback
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	// Read the current measurement here

	// Publish a message
	std_msgs::Float32 msg;
	msg.data = 0.0;
	m_current_publisher.publish(msg);
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "template_i2c_internal");
	ros::NodeHandle nodeHandle("~");
	// Initialise a publisher
	//ros::Publisher current_measurement_publisher = nodeHandle.advertise<std_msgs::UInt16>("current_measurement", 10, false);
	// Initialise a subscriber
	ros::Subscriber set_motor_duty_cycle_subscriber = nodeHandle.subscribe("set_motor_duty_cycle", 1, templateSubscriberCallback);
	// Initialise a subscriber for the servo driver
	ros::Subscriber set_servo_pulse_width_subscriber = nodeHandle.subscribe("set_servo_pulse_width", 1, templateSubscriberCallback);

	// Initialise a publisher for the current
	// sensor measurement
	m_current_publisher = nodeHandle.advertise<std_msgs::Float32>("great_topic", 10, false);
	// Initialise a timer
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(0.02), timerCallbackForPublishing, false);

	// Open the I2C device
	// > Note that the I2C driver is already instantiated
	//   as a member variable of this node
	bool open_success = m_i2c_driver.open_i2c_device();

	// Display the status
	if (!open_success)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
	}

	// Note:
	// > The drivers are already instantiated as
	//   member variables of this node for:
	//   > Each of the Pololu simple motor controllers (SMC)
	//   > The servo driver



	// SET THE CONFIGURATION OF EACH MOTOR CONTROLLER

	// Specify the various limits
	int new_current_limit_in_milliamps = 5000;
	int new_max_speed_limit = 2560;
	int new_max_accel_limit = 1;
	int new_max_decel_limit = 5;

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	Pololu_SMC_G2 * pololu_smc_pointer;

	// Initialise each of the Pololu SMC objects
	// with the limits specified above.

	// Iterate over the pololu objects
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		// Point to the appropriate motor controller
		if (i_smc==0)
			pololu_smc_pointer = &m_pololu_smc_left;
		else
			pololu_smc_pointer = &m_pololu_smc_right;

		// Display the object about to be initialised
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Now initialising SMC with I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );

		// Call the Pololu SMC initialisation function
		bool verbose_display_for_SMC_init = false;
		bool result_smc_init = pololu_smc_pointer->initialise_with_limits(new_current_limit_in_milliamps,new_max_speed_limit,new_max_accel_limit,new_max_decel_limit,verbose_display_for_SMC_init);

		// Display if an error occurred
		if (!result_smc_init)
		{
			ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED - while initialising SMC with I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );
		}
	}


	// SET THE CONFIGURATION OF THE PERVO DRIVER

	// Specify the frequency of the servo driver
	float new_frequency_in_hz = 50.0;

	// Call the Servo Driver initialisation function
	bool verbose_display_for_servo_driver_init = false;
	bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);

	// Display if an error occurred
	if (!result_servo_init)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
	}

	// Spin as a single-threaded node
	ros::spin();

	// Close the I2C device
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[TEMPLATE I2C INTERNAL] Successfully closed device named " << m_i2c_driver.get_device_name());
	}

	return 0;
}
