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
// Template node for mointoring edge events on a GPIO pin
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <gpiod.h>

// Includes required for threading
// > Mutex for handling shared variable
#include <mutex>
// > Include only one of the following two options:
#include <thread>
//#include <boost/thread/thread.hpp>

// Include the asclinic message types
#include "asclinic_pkg/LeftAndRightInt.h"

// Namespacing the package
using namespace asclinic_pkg;





// MEMBER VARIABLES FOR THIS NODE:
// > Publisher and timer for the current
//   encoder counts
ros::Publisher m_encoder_counts_publisher;
ros::Timer m_timer_for_publishing;

// > For sharing the encoder counts between nodes
int m_encoder_counts_for_motor_left  = 0;
int m_encoder_counts_for_motor_right = 0;

// > Mutex for preventing multiple-access of shared variables
std::mutex m_counting_mutex;

// > The line numbers to read
int m_line_number_for_motor_left_channel_a = 133;
int m_line_number_for_motor_right_channel_a = 105;

// > The "delta t" used for the frequency of publishing encoder counts
float m_delta_t_for_publishing_counts = 0.1;

// > NOTE: The following variables are purely for
//   the convenience of testing.
float m_time_in_seconds_to_drive_motors = 1.0;
ros::Publisher m_motor_pwm_publisher;
float m_drive_motor_target_speed = 20.0;




// Respond to timer callback
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	// Get the current counts into a local variable
	// > And reset the shared counts variable to zero
	int counts_motor_left_local_copy;
	int counts_motor_right_local_copy;
	m_counting_mutex.lock();
	counts_motor_left_local_copy  = m_encoder_counts_for_motor_left;
	counts_motor_right_local_copy = m_encoder_counts_for_motor_right;
	m_encoder_counts_for_motor_left  = 0;
	m_encoder_counts_for_motor_right = 0;
	m_counting_mutex.unlock();

	// Publish a message
	LeftAndRightInt msg;
	msg.left  = counts_motor_left_local_copy;
	msg.right = counts_motor_right_local_copy;
	m_encoder_counts_publisher.publish(msg);


	// NOTE: the remainder of this function is
	// purely for the convenience of testing.
	static bool did_start_motors = false;
	static bool did_finish_test = false;
	static bool did_display_cum_sum = false;
	static float elapsed_time_in_seconds = 0.0;
	static int cum_sum_left = 0;
	static int cum_sum_right = 0;
	// Add the counts
	cum_sum_left  += counts_motor_left_local_copy;
	cum_sum_right += counts_motor_right_local_copy;// Increment the time
	// Increment the time
	elapsed_time_in_seconds += m_delta_t_for_publishing_counts;
	// Start the motors after a few seconds
	if ( !(did_start_motors) && (elapsed_time_in_seconds>=2.0) )
	{
		// Publish message to start the motors
		std_msgs::Float32 target_speed_msg;
		target_speed_msg.data = m_drive_motor_target_speed;
		m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_start_motors = true;
	}
	// Stop the motors after "m_time_in_seconds_to_drive_motors"
	if ( !(did_finish_test) && (elapsed_time_in_seconds>=(2.0+m_time_in_seconds_to_drive_motors)) )
	{
		// Publish message to stop the motors
		std_msgs::Float32 target_speed_msg;
		target_speed_msg.data = 0.0;
		m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_finish_test = true;
	}
	// Display the cumulative cum
	if ( !(did_display_cum_sum) && (elapsed_time_in_seconds>=(2.0+2.0+m_time_in_seconds_to_drive_motors)) )
	{
		ROS_INFO_STREAM("[TEMPLATE ENCODER READ MULTI THREADED] cumulative sum left = " << cum_sum_left << ", right = " << cum_sum_right);
		// Update the flag
		did_display_cum_sum = true;
	}
}




void encoderCountingThreadMain()
{
	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this
	//   is "/dev/gpiochip0"
	const char * gpio_chip_name = "/dev/gpiochip0";

	// Make a local copy of the line number member variables
	int line_number_left_a  = m_line_number_for_motor_left_channel_a;
	int line_number_right_a = m_line_number_for_motor_right_channel_a;

	// Initialise a GPIO chip, line, and event objects
	struct gpiod_chip *chip;
	struct gpiod_line *line_left_a;
	struct gpiod_line *line_right_a;
	struct gpiod_line_bulk line_bulk;
	struct gpiod_line_event event;
	struct gpiod_line_bulk event_bulk;

	// Specify the timeout specifications
	// > The first entry is seconds
	// > The second entry is nano-seconds
	struct timespec timeout_spec = { 0, 10000000 };
	
	// Intialise a variable for the flags returned
	// by GPIO calls
	int returned_wait_flag;
	int returned_read_flag;

	// Get and print the value of the GPIO line
	// > Note: the third argument to "gpiod_ctxless_get_value"
	//   is an "active_low" boolean input argument.
	//   If true, this indicate to the function that active state
	//   of this line is low.
	int value;
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_a, false, "foobar");
	ROS_INFO_STREAM("[TEMPLATE ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_a << " returned value = " << value);
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_a, false, "foobar");
	ROS_INFO_STREAM("[TEMPLATE ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_a << " returned value = " << value);

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO lines
	line_left_a  = gpiod_chip_get_line(chip,line_number_left_a);
	line_right_a = gpiod_chip_get_line(chip,line_number_right_a);
	// Initialise the line bulk
	gpiod_line_bulk_init(&line_bulk);
	// Add the lines to the line bulk
	gpiod_line_bulk_add(&line_bulk, line_left_a);
	gpiod_line_bulk_add(&line_bulk, line_right_a);

	// Display the status
	ROS_INFO_STREAM("[TEMPLATE ENCODER READ MULTI THREADED] Chip " << gpio_chip_name << " opened and lines " << line_number_left_a << " and " << line_number_right_a << " retrieved");

	// Request the line events to be mointored
	// > Note: only one of these should be uncommented
	//gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
	gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
	//gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");

	// Display the line event values for rising and falling
	ROS_INFO_STREAM("[TEMPLATE ENCODER READ MULTI THREADED] The constants defined for distinguishing line events are:, GPIOD_LINE_EVENT_RISING_EDGE = " << GPIOD_LINE_EVENT_RISING_EDGE << ", and GPIOD_LINE_EVENT_FALLING_EDGE = " << GPIOD_LINE_EVENT_FALLING_EDGE);

	// Enter a loop that endlessly monitors the encoders
	while (true)
	{

		// Monitor for the requested events on the GPIO line bulk
		// > Note: the function "gpiod_line_event_wait" returns:
		//    0  if wait timed out
		//   -1  if an error occurred
		//    1  if an event occurred.
		returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);

		// Respond based on the the return flag
		if (returned_wait_flag == 1)
		{
			// Get the number of events that occurred
			int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

			// Lock the mutex while before counting the events
			m_counting_mutex.lock();

			// Iterate over the event
			for (int i_event = 0; i_event < num_events_during_wait; i_event++)
			{
				// Get the line handle for this event
				struct gpiod_line *line_handle = gpiod_line_bulk_get_line(&event_bulk, i_event);

				// Get the number of this line
				unsigned int this_line_number = gpiod_line_offset(line_handle);

				// Read the event on the GPIO line
				// > Note: the function "gpiod_line_event_read" returns:
				//    0  if the event was read correctly
				//   -1  if an error occurred
				returned_read_flag = gpiod_line_event_read(line_handle,&event);

				// Respond based on the the return flag
				if (returned_read_flag == 0)
				{
					// Increment the respective count
					if (this_line_number == line_number_left_a)
						m_encoder_counts_for_motor_left++;
					if (this_line_number == line_number_right_a)
						m_encoder_counts_for_motor_right++;

				} // END OF: "if (returned_read_flag == 0)"

			} // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

			// Unlock the mutex
			m_counting_mutex.unlock();

		} // END OF: "if (returned_wait_flag == 1)"
	} // END OF: "while (true)"

	// Close the GPIO chip
	gpiod_chip_close(chip);
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "template_encoder_read_multi_threaded");
	ros::NodeHandle nodeHandle("~");

	// Get the GPIO line number to monitor
	// Notes:
	// > If you look at the "template_encoder.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number_for_motor_left_channel_a"
	//           value  = 133
	//       />
	// > These lines of code add a parameter named to this node
	//   with the parameter name: "line_number_for_motor_left_channel_a"
	// > Thus, to access this parameter, we first get a handle to
	//   this node within the namespace that it was launched.
	//
	// Get the line number parameters:
	// > For channel A of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_a", m_line_number_for_motor_left_channel_a) )
	{
		// Display an error message
		ROS_INFO("[TEMPLATE ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_a\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_a", m_line_number_for_motor_right_channel_a) )
	{
		// Display an error message
		ROS_INFO("[TEMPLATE ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_a\" parameter. Using default value instead.");
	}
	// > Display the line numbers being monitored
	ROS_INFO_STREAM("[TEMPLATE ENCODER READ MULTI THREADED] Will monitor line_numbers = " << m_line_number_for_motor_left_channel_a << " and " << m_line_number_for_motor_right_channel_a);

	// Get the "detla t" parameter for the publishing frequency
	if ( !nodeHandle.getParam("delta_t_for_publishing_counts", m_delta_t_for_publishing_counts) )
	{
		// Display an error message
		ROS_INFO("[TEMPLATE ENCODER READ MULTI THREADED] FAILED to get \"delta_t_for_publishing_counts\" parameter. Using default value instead.");
	}

	// Initialise a publisher for the encoder counts
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_encoder_counts_publisher = nodeHandle.advertise<LeftAndRightInt>("encoder_counts", 10, false);

	// Initialise a timer
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(m_delta_t_for_publishing_counts), timerCallbackForPublishing, false);

	// Get the parameter for how long to drive the motors
	// NOTE: this parameter is purely for the convenience of testing.
	if ( !nodeHandle.getParam("time_in_seconds_to_drive_motors", m_time_in_seconds_to_drive_motors) )
		ROS_INFO("[TEMPLATE ENCODER READ MULTI THREADED] FAILED to get \"time_in_seconds_to_drive_motors\" parameter. Using default value instead.");

	// Initialise a publisher for commanding the motors
	// NOTE: this publisher and what it is used for is
	// purely for the convenience of testing.
	std::string temp_namespace = ros::this_node::getNamespace();
	std::string temp_namespace_to_i2c_internal = temp_namespace + "/template_i2c_internal";
	ros::NodeHandle nodeHandle_to_i2c_internal(temp_namespace_to_i2c_internal);
	m_motor_pwm_publisher = nodeHandle_to_i2c_internal.advertise<std_msgs::Float32>("set_motor_duty_cycle", 10, false);

	// Get the parameter for target speed when driving the motors
	// NOTE: this parameter is purely for the convenience of testing.
	if ( !nodeHandle.getParam("drive_motor_target_speed", m_drive_motor_target_speed) )
		ROS_INFO("[TEMPLATE ENCODER READ MULTI THREADED] FAILED to get \"drive_motor_target_speed\" parameter. Using default value instead.");

	// Create thread for counting the encoder events
	std::thread encoder_counting_thread (encoderCountingThreadMain);
	//boost::thread encoder_counting_thread(encoderCountingThreadMain);

	// Spin the node
	ros::spin();

	// Join back the encoder counting thread
	encoder_counting_thread.join();

	return 0;
}
