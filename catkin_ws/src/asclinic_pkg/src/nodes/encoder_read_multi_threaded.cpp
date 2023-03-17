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
// Node for monitoring GPIO pins connected to wheel encoders
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
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/LeftRightFloat32.h"

// Namespacing the package
//using namespace asclinic_pkg;





// MEMBER VARIABLES FOR THIS NODE:
// > Publisher and timer for the current
//   encoder counts
ros::Publisher m_encoder_counts_publisher;
ros::Timer m_timer_for_publishing;

// > For sharing the encoder counts between nodes
int m_encoder_counts_for_motor_left_a  = 0;
int m_encoder_counts_for_motor_left_b  = 0;
int m_encoder_counts_for_motor_right_a = 0;
int m_encoder_counts_for_motor_right_b = 0;

// > Mutex for preventing multiple-access of shared variables
std::mutex m_counting_mutex;

// > The "gpiochip" number
int m_gpiochip_number = 1;

// > The line numbers to read
int m_line_number_for_motor_left_channel_a  = 105;
int m_line_number_for_motor_left_channel_b  = 106;
int m_line_number_for_motor_right_channel_a =  84;
int m_line_number_for_motor_right_channel_b = 130;

// > Boolean flag for when to stop counting
bool encoder_thread_should_count = true;

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
	int counts_motor_left_a_local_copy;
	int counts_motor_left_b_local_copy;
	int counts_motor_right_a_local_copy;
	int counts_motor_right_b_local_copy;
	m_counting_mutex.lock();
	counts_motor_left_a_local_copy  = m_encoder_counts_for_motor_left_a;
	counts_motor_left_b_local_copy  = m_encoder_counts_for_motor_left_b;
	counts_motor_right_a_local_copy = m_encoder_counts_for_motor_right_a;
	counts_motor_right_b_local_copy = m_encoder_counts_for_motor_right_b;
	m_encoder_counts_for_motor_left_a  = 0;
	m_encoder_counts_for_motor_left_b  = 0;
	m_encoder_counts_for_motor_right_a = 0;
	m_encoder_counts_for_motor_right_b = 0;
	m_counting_mutex.unlock();

	// Publish a message
	asclinic_pkg::LeftRightInt32 msg;
	msg.left  = counts_motor_left_a_local_copy  + counts_motor_left_b_local_copy;
	msg.right = counts_motor_right_a_local_copy + counts_motor_right_b_local_copy;
	m_encoder_counts_publisher.publish(msg);


	// NOTE: the remainder of this function is
	// purely for the convenience of testing.
	static bool did_start_motors = false;
	static bool did_finish_test = false;
	static bool did_display_cum_sum = false;
	static float elapsed_time_in_seconds = 0.0;
	static int cum_sum_left_a = 0;
	static int cum_sum_left_b = 0;
	static int cum_sum_right_a = 0;
	static int cum_sum_right_b = 0;
	// Add the counts
	cum_sum_left_a  += counts_motor_left_a_local_copy;
	cum_sum_left_b  += counts_motor_left_b_local_copy;
	cum_sum_right_a += counts_motor_right_a_local_copy;
	cum_sum_right_b += counts_motor_right_b_local_copy;
	// Increment the time
	elapsed_time_in_seconds += m_delta_t_for_publishing_counts;
	// Start the motors after a few seconds
	if ( !(did_start_motors) && (elapsed_time_in_seconds>=2.0) )
	{
		// Publish message to start the motors
		asclinic_pkg::LeftRightFloat32 target_speed_msg;
		target_speed_msg.left  = m_drive_motor_target_speed;
		target_speed_msg.right = m_drive_motor_target_speed;
		m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_start_motors = true;
	}
	// Stop the motors after "m_time_in_seconds_to_drive_motors"
	if ( !(did_finish_test) && (elapsed_time_in_seconds>=(2.0+m_time_in_seconds_to_drive_motors)) )
	{
		// Publish message to stop the motors
		asclinic_pkg::LeftRightFloat32 target_speed_msg;
		target_speed_msg.left  = 0.0;
		target_speed_msg.right = 0.0;
		m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_finish_test = true;
	}
	// Display the cumulative cum
	if ( !(did_display_cum_sum) && (elapsed_time_in_seconds>=(2.0+2.0+m_time_in_seconds_to_drive_motors)) )
	{
		ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] cumulative sum left (A,B) = ( " << cum_sum_left_a << " , " << cum_sum_left_b << " ), right (A,B) = ( " << cum_sum_right_a << " , " << cum_sum_right_b << " )");
		// Update the flag
		did_display_cum_sum = true;
		// Update the flag to end the encoder thread
		encoder_thread_should_count = false;
	}
}




void encoderCountingThreadMain()
{
	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this
	//   is "/dev/gpiochip1"
	std::stringstream temp_string_stream;
	temp_string_stream << "/dev/gpiochip" << m_gpiochip_number;
	const char * gpio_chip_name = temp_string_stream.str().c_str();

	// Make a local copy of the line number member variables
	int line_number_left_a  = m_line_number_for_motor_left_channel_a;
	int line_number_left_b  = m_line_number_for_motor_left_channel_b;
	int line_number_right_a = m_line_number_for_motor_right_channel_a;
	int line_number_right_b = m_line_number_for_motor_right_channel_b;

	// Initialise a GPIO chip, line, and event objects
	struct gpiod_chip *chip;
	struct gpiod_line *line_left_a;
	struct gpiod_line *line_left_b;
	struct gpiod_line *line_right_a;
	struct gpiod_line *line_right_b;
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
	// > For left motor channel A
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_a, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_a << " returned value = " << value);
	// > For left motor channel B
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_b, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_b << " returned value = " << value);
	// > For right motor channel A
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_a, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_a << " returned value = " << value);
	// > For right motor channel B
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_b, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_b << " returned value = " << value);

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO lines
	line_left_a  = gpiod_chip_get_line(chip,line_number_left_a);
	line_left_b  = gpiod_chip_get_line(chip,line_number_left_b);
	line_right_a = gpiod_chip_get_line(chip,line_number_right_a);
	line_right_b = gpiod_chip_get_line(chip,line_number_right_b);
	// Initialise the line bulk
	gpiod_line_bulk_init(&line_bulk);
	// Add the lines to the line bulk
	gpiod_line_bulk_add(&line_bulk, line_left_a);
	gpiod_line_bulk_add(&line_bulk, line_left_b);
	gpiod_line_bulk_add(&line_bulk, line_right_a);
	gpiod_line_bulk_add(&line_bulk, line_right_b);

	// Display the status
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Chip " << gpio_chip_name << " opened and lines " << line_number_left_a << ", " << line_number_left_b << ", " << line_number_right_a << " and " << line_number_right_b << " retrieved");

	// Request the line events to be mointored
	// > Note: only one of these should be uncommented
	//gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
	gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
	//gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");

	// Display the line event values for rising and falling
	//ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] The constants defined for distinguishing line events are:, GPIOD_LINE_EVENT_RISING_EDGE = " << GPIOD_LINE_EVENT_RISING_EDGE << ", and GPIOD_LINE_EVENT_FALLING_EDGE = " << GPIOD_LINE_EVENT_FALLING_EDGE);

	// Enter a loop that endlessly monitors the encoders
	while (encoder_thread_should_count)
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
						m_encoder_counts_for_motor_left_a++;
					else if (this_line_number == line_number_left_b)
						m_encoder_counts_for_motor_left_b++;
					else if (this_line_number == line_number_right_a)
						m_encoder_counts_for_motor_right_a++;
					else if (this_line_number == line_number_right_b)
						m_encoder_counts_for_motor_right_b++;

				} // END OF: "if (returned_read_flag == 0)"

			} // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

			// Unlock the mutex
			m_counting_mutex.unlock();

		} // END OF: "if (returned_wait_flag == 1)"
	} // END OF: "while (true)"

	// Release the lines
	gpiod_line_release_bulk(&line_bulk);
	// Close the GPIO chip
	gpiod_chip_close(chip);
	// Inform the user
	ROS_INFO("[ENCODER READ MULTI THREADED] Lines released and GPIO chip closed");
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "encoder_read_multi_threaded");
	ros::NodeHandle nodeHandle("~");

	// Get the GPIO line number to monitor
	// Notes:
	// > If you look at the "encoder.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number_for_motor_left_channel_a"
	//           value  = 105
	//       />
	// > These lines of code add a parameter named to this node
	//   with the parameter name: "line_number_for_motor_left_channel_a"
	// > Thus, to access this parameter, we first get a handle to
	//   this node within the namespace that it was launched.
	//

	// Get the "gpiochip" number parameter:
	if ( !nodeHandle.getParam("gpiochip_number", m_gpiochip_number) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"gpiochip_number\" parameter. Using default value instead.");
	}

	// Get the line number parameters:
	// > For channel A of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_a", m_line_number_for_motor_left_channel_a) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_a\" parameter. Using default value instead.");
	}
	// > For channel B of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_b", m_line_number_for_motor_left_channel_b) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_b\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_a", m_line_number_for_motor_right_channel_a) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_a\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_b", m_line_number_for_motor_right_channel_b) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_b\" parameter. Using default value instead.");
	}
	// > Display the line numbers being monitored
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line_numbers = " << m_line_number_for_motor_left_channel_a << ", " << m_line_number_for_motor_left_channel_b << ", " << m_line_number_for_motor_right_channel_a << ", and " << m_line_number_for_motor_right_channel_b);

	// Get the "detla t" parameter for the publishing frequency
	if ( !nodeHandle.getParam("delta_t_for_publishing_counts", m_delta_t_for_publishing_counts) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"delta_t_for_publishing_counts\" parameter. Using default value instead.");
	}

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Initialise a publisher for the encoder counts
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_encoder_counts_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightInt32>("encoder_counts", 10, false);

	// Initialise a timer
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(m_delta_t_for_publishing_counts), timerCallbackForPublishing, false);

	// Get the parameter for how long to drive the motors
	// NOTE: this parameter is purely for the convenience of testing.
	if ( !nodeHandle.getParam("time_in_seconds_to_drive_motors", m_time_in_seconds_to_drive_motors) )
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"time_in_seconds_to_drive_motors\" parameter. Using default value instead.");

	// Initialise a publisher for commanding the motors
	// NOTE: this publisher and what it is used for is
	// purely for the convenience of testing.
	m_motor_pwm_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("set_motor_duty_cycle", 10, false);

	// Get the parameter for target speed when driving the motors
	// NOTE: this parameter is purely for the convenience of testing.
	if ( !nodeHandle.getParam("drive_motor_target_speed", m_drive_motor_target_speed) )
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"drive_motor_target_speed\" parameter. Using default value instead.");

	// Create thread for counting the encoder events
	std::thread encoder_counting_thread (encoderCountingThreadMain);
	//boost::thread encoder_counting_thread(encoderCountingThreadMain);

	// Spin the node
	ros::spin();

	// Set the flag to stop counting
	encoder_thread_should_count = false;

	// Join back the encoder counting thread
	encoder_counting_thread.join();

	return 0;
}
