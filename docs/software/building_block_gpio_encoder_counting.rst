.. _building-block-gpio-encoder-counting:

Counting encoder ticks via GPIO pins
====================================

You do **not** require ROS to be able to read a wheel encoder via GPIO pins, though ROS makes it easy to send (i.e., publish) the encoder data to other building blocks of your autonomous system. Hence this guide first builds up a stand-alone C++ code that counts the ticks of multiple wheel encoder channels; then as a final step, the code is put into a ROS node, i.e., an encoder counter node.

Requirements for this page:

* Basic proficiency with C++
* Familiarity with the the structure of a :ref:`ROS node that spins at a fixed frequency <ros-code-spin-at-frequency>`. 

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2



TL;DR (ROS encoder counter node)
********************************

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail.

Take the stand-alone C++ encoder counter code from below (i.e., from the :ref:`building-block-gpio-encoder-counting-tldr-cpp` section), and follow the changes described in the :ref:`building-block-gpio-encoder-counting-put-into-ros` section.



.. _building-block-gpio-encoder-counting-tldr-cpp:

TL;DR (stand-alone C++ encoder counter)
***************************************

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail.

The full contents for a stand alone C++ encoder counter is just below, which you can compile and execute using the commands:

.. code-block:: bash

  g++ -std=c++11 -o encoder_counter encoder_counter.cpp -lpthread -lgpiod
  ./encoder_counter

The executable will run for the few seconds during which time it is counting events on the specified GPIO lines.


.. toggle::

  .. code-block:: cpp

    // Includes required for threading
    #include <mutex>
    #include <thread>
    #include <unistd.h>
    #include <iostream>
    #include <gpiod.h>

    // Declare the member variables
    // > For sharing the encoder counts between threads
    int m_counts_for_m1a = 0;
    int m_counts_for_m1b = 0;
    int m_counts_for_m2a = 0;
    int m_counts_for_m2b = 0;
    // > Mutex for preventing multiple-access of shared variables
    std::mutex m_counting_mutex;
    // > Boolean flag for when to stop counting
    bool m_threads_should_count = true;

    // Declare the function prototypes
    // > For the thread that reads (and displays) the current counts
    void threadForReading();
    // > For the thread that counts encoder events
    void threadForCounting();



    // Function implementation
    void threadForCounting()
    {
      // Specify the gpio chip name of the GPIO interface
      const char * gpio_chip_name = "/dev/gpiochip1";

      // Specify the line numbers where the encoder channels are connected
      int line_number_for_m1a = 105;
      int line_number_for_m1b = 106;
      int line_number_for_m2a =  84;
      int line_number_for_m2b = 130;

      // Initialise a GPIO chip, line, and event objects
      struct gpiod_chip *chip;
      struct gpiod_line *line_m1a;
      struct gpiod_line *line_m1b;
      struct gpiod_line *line_m2a;
      struct gpiod_line *line_m2b;
      struct gpiod_line_bulk line_bulk;
      struct gpiod_line_event event;
      struct gpiod_line_bulk event_bulk;

      // Specify the timeout specifications
      // > The first entry is seconds
      // > The second entry is nano-seconds
      struct timespec timeout_spec = { 0, 10000000 };

      // Intialise a variable for the flags returned by GPIO calls
      int returned_wait_flag;
      int returned_read_flag;

      // Open the GPIO chip
      chip = gpiod_chip_open(gpio_chip_name);
      // Retrieve the GPIO lines
      line_m1a  = gpiod_chip_get_line(chip,line_number_for_m1a);
      line_m1b  = gpiod_chip_get_line(chip,line_number_for_m1b);
      line_m2a = gpiod_chip_get_line(chip,line_number_for_m2a);
      line_m2b = gpiod_chip_get_line(chip,line_number_for_m2b);
      // Initialise the line bulk
      gpiod_line_bulk_init(&line_bulk);
      // Add the lines to the line bulk
      gpiod_line_bulk_add(&line_bulk, line_m1a);
      gpiod_line_bulk_add(&line_bulk, line_m1b);
      gpiod_line_bulk_add(&line_bulk, line_m2a);
      gpiod_line_bulk_add(&line_bulk, line_m2b);

      // Display the status
      std::cout << "[ENCODER COUNTER] Chip " << gpio_chip_name << " opened and lines " << line_number_for_m1a << ", " << line_number_for_m1b << ", " << line_number_for_m2a << " and " << line_number_for_m2a << " retrieved";

      // Request the line events to be monitored
      // > Note: only one of these should be uncommented
      gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
      //gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
      //gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");


      // Enter a loop that monitors the encoders
      while (m_threads_should_count)
      {
        // Monitor for the requested events on the GPIO line bulk
        // > Note: the function "gpiod_line_event_wait" returns:
        //    0  if wait timed out
        //   -1  if an error occurred
        //    1  if an event occurred.
        returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);

        // Lock the mutex before processing the events
        m_counting_mutex.lock();

        // Respond based on the the return flag
        if (returned_wait_flag == 1)
        {
          // Get the number of events that occurred
          int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

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
              if (this_line_number == line_number_for_m1a)
                m_counts_for_m1a++;
              else if (this_line_number == line_number_for_m1b)
                m_counts_for_m1b++;
              else if (this_line_number == line_number_for_m2a)
                m_counts_for_m2a++;
              else if (this_line_number == line_number_for_m2b)
                m_counts_for_m2b++;

            } // END OF: "if (returned_read_flag == 0)"

          } // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

        } // END OF: "if (returned_wait_flag == 1)"

        // Get a local copy of the flag
        bool should_count_local_copy = m_threads_should_count;

        // Unlock the mutex
        m_counting_mutex.unlock();

        // Break if necessary
        if (!should_count_local_copy)
          break;

      } // END OF: "while (true)"

      // Release the lines
      gpiod_line_release_bulk(&line_bulk);
      // Close the GPIO chip
      gpiod_chip_close(chip);
      // Inform the user
      std::cout << "[ENCODER COUNTER] Lines released and GPIO chip closed";

    } // END OF: "void threadForCounting()"



    // Function implementation
    void threadForReading()
    {
      static int cumulative_counts_for_m1 = 0;
      static int cumulative_counts_for_m2 = 0;

      while(m_threads_should_count)
      {
        // Get a local copy of the variables
        m_counting_mutex.lock();
        int counts_for_m1a_local_copy = m_counts_for_m1a;
        int counts_for_m1b_local_copy = m_counts_for_m1b;
        int counts_for_m2a_local_copy = m_counts_for_m2a;
        int counts_for_m2b_local_copy = m_counts_for_m2b;
        bool should_count_local_copy  = m_threads_should_count;
        // Reset the shared counting variables to zero
        m_counts_for_m1a = 0;
        m_counts_for_m1b = 0;
        m_counts_for_m2a = 0;
        m_counts_for_m2b = 0;
        m_counting_mutex.unlock();

        // Add the counts to the cumulative total
        cumulative_counts_for_m1 += (counts_for_m1a_local_copy + counts_for_m1b_local_copy);
        cumulative_counts_for_m2 += (counts_for_m2a_local_copy + counts_for_m2b_local_copy);

        // Display the cumulative counts
        printf("cumulative counts = [%d, %d]\n", cumulative_counts_for_m1, cumulative_counts_for_m2);

        // Break if necessary
        if (!should_count_local_copy)
          break;

        // Sleep for a bit
        usleep(100000);
      }
    }



    // Main function
    int main(int argc, char* argv[])
    {
      // Set the flag that threads should count
      m_threads_should_count = true;

      // Create the threads
      std::thread counting_thread (threadForCounting);
      std::thread reading_thread (threadForReading);

      // Spin the main function for a bit
      sleep(3);

      // Set the flag to finish counting
      m_counting_mutex.lock();
      m_threads_should_count = false;
      m_counting_mutex.unlock();

      // Join back the threads
      counting_thread.join();
      reading_thread.join();
    }





High level description
**********************

Under normal operating conditions, many wheel encoders generate pulses at up to 10 kHz; where the exact rate depends on the encoder resolution, gearbox ratio, and wheel speed. The key point is that we need to think carefully about the limitations and overheads of using a non real-time operating system (i.e., Ubuntu) and a user-space GPIO library (i.e., :code:`libgpiod` or :code:`sysfs`). To achieve the most reliable counting of encoder pulses at 10 kHz or more.

To achieve reliable counting we use a multi-threaded implementation where:

* One thread monitors the GPIO pins connected to each encoder channel and counts all the rising and/or falling edges. This thread should be programmed in a fashion that has minimal over-head, i.e., no "fluff" in the code.

* Another thread reads the the current value of the encoder counts at a fixed frequency and does something useful with the data, e.g., displays the counts value on the screen, publishes the data, performs odometry computations.

* The variables that store the current value of the encoder counts are the only variables shared between the two threads, hence a mutex is used to serialise access to these variables.


Standard threads, boost threads or p threads - which one to use
***************************************************************

There are three main libraries for multi-threading in C++ and an in depth discussion is beyond the scope of this wiki page. **As a rule-of-thumb for any multi-threaded code, the standard threads option (**:code:`std::thread` **) should be your default choice.** Only consider one of the other options if your use-case requires features not available with :code:`std::thread`. As encoder counting with multiple threads is a simple use-case, we use :code:`std::thread`.

To investigate the three options further, simply google it, and likely you will find the following summary and much much more:

* :code:`std::thread`

  * Is available on most all platforms.
  * Requires C++11 or newer compiler.
  * Provides only basic multi-threading features.

* :code:`boost::thread`

  * Is available on most all platforms.
  * Is supported by older compilers (i.e., older than C++11)
  * May require external dependencies, i.e., not install by default on some platforms.
  * Offers similar features to :code:`std::thread`

* :code:`pthread`

  * Offers more features that the other two options, for example, scheduling policies.
  * Only works on POSIX systems, i.e., does not work on Windows.

..
  //boost::thread encoder_counting_thread(encoderCountingThreadMain);
  //#include <boost/thread/thread.hpp>



.. _building-block-gpio-encoder-counting-boiler-plate:

Prototypical multi-threaded code structure
******************************************

We start from a "boiler plate" template that captures our key multi-threaded structure, i.e., separate threads for counting and reading, and a main thread that flags when to stop.

Piece together the following to build up the "boiler plate" template code:


| **Step 1**
| Include the headers required for launching threads (i.e., :code:`#include <thread>`), and for using mutex object (i.e., :code:`#include <mutex>`).

  .. code-block:: cpp

    // Includes required for threading
    #include <mutex>
    #include <thread>

    // Other includes
    #include <unistd.h>
    #include <iostream>

| **Step 2**
| Declare the variables that are to be shared across all functions in this code. Also this is not written as a C++ class, we use the :code:`m_` prefix to indicate "member" variables that are accessible from all functions.

  .. code-block:: cpp

    // Declare the member variables
    // > For sharing the encoder counts between threads
    int m_counts_for_m1a = 0;
    int m_counts_for_m1b = 0;
    int m_counts_for_m2a = 0;
    int m_counts_for_m2b = 0;
    // > Mutex for preventing multiple-access of shared variables
    std::mutex m_counting_mutex;
    // > Boolean flag for when to stop counting
    bool m_threads_should_count = true;


| **Step 3**
| Declare the function prototypes for the functions.

  .. code-block:: cpp

    // Declare the function prototypes
    // > For the thread that reads (and displays) the current counts
    void threadForReading();
    // > For the thread that counts encoder events
    void threadForCounting();

| **Step 4**
| Implement the function for reading and displaying the current counts in a thread-safe fashion:
|  a) Enter a while loop that continues while the flag indicates to do so.
|  b) Lock the mutex, then take a local copy the shared variables, then unlock the mutex.
|  c) Display the local copy of the shared variables.
|  d) Sleep for a short time so that the while loop proceed at an approximate fixed frequency.

  .. code-block:: cpp

    // Function implementation
    void threadForReading()
    {
      while(m_threads_should_count)
      {
        // Get a local copy of the variables
        m_counting_mutex.lock();
        int counts_for_m1a_local_copy = m_counts_for_m1a;
        int counts_for_m1b_local_copy = m_counts_for_m1b;
        int counts_for_m2a_local_copy = m_counts_for_m2a;
        int counts_for_m2b_local_copy = m_counts_for_m2b;
        bool should_count_local_copy  = m_threads_should_count;
        m_counting_mutex.unlock();

        // Display the counts
        printf("counts = [%d, %d, %d, %d]\n", counts_for_m1a_local_copy, counts_for_m1b_local_copy, counts_for_m2a_local_copy, counts_for_m2b_local_copy);

        // Break if necessary
        if (!should_count_local_copy)
          break;

        // Sleep for a bit
        usleep(100000);
      }
    }

  **Here it is important to understand that when the mutex is locked, then the threads are potentially serialised** (i.e., the other thread pauses execution if it reaches its mutex lock line of code during this period). Hence only minimal operations are performed while the mutex is locked, i.e., quickly take a local copy of the variables.

| **Step 5**
| Implement the function for counting the encoder events:
|  a) Enter a while loop that continues while the flag indicates to do so.
|  b) Lock the mutex.
|  c) Update the shared variable for encoder counts. **Note:** for counting actual encoder events, the following sections develop the code that needs to be inserted here.
|  d) Take a local copy the flag for whether to keeping counting or not
|  e) Unlock the mutex.
|  f) Sleep for a short time so that the while loop proceed at an approximate fixed frequency.

  .. code-block:: cpp

    // Function implementation
    void threadForCounting()
    {
      while(m_threads_should_count)
      {
        // Do some fake counting for the purpose of this template
        m_counting_mutex.lock();
        m_counts_for_m1a++;
        m_counts_for_m1b++;
        m_counts_for_m2a++;
        m_counts_for_m2b++;
        bool should_count_local_copy = m_threads_should_count;
        m_counting_mutex.unlock();

        // Break if necessary
        if (!should_count_local_copy)
          break;

        // Sleep for a bit for the purpose of this template
        usleep(10000);
      }
    }

    **Reiterating: it is important to understand that when the mutex is locked, then the threads are potentially serialised.**

| **Step 6**
| Implement the main function to:
|  a) Launch the two threads.
|  b) Sleep the main function for a few seconds.
|  c) Set the flag to stop counting
|  d) Join the threads back, i.e., the execution of the main function pauses here until the respective thread has reached the end of its respective function.

  .. code-block:: cpp

    // Main function
    int main(int argc, char* argv[])
    {
      // Set the flag that threads should count
      m_threads_should_count = true;

      // Create the threads
      std::thread counting_thread (threadForCounting);
      std::thread reading_thread (threadForReading);

      // Spin the main function for a bit
      sleep(3);

      // Set the flag to finish counting
      m_counting_mutex.lock();
      m_threads_should_count = false;
      m_counting_mutex.unlock();

      // Join back the threads
      counting_thread.join();
      reading_thread.join();
    }



Compile and run (before adding more complexity)
***********************************************

To check that the multi-threaded development pipeline is working for you, it is a good idea to compile and run the prototypical starting point given in the section above.

* Create a file with the contents given in the section above, for example with filename: :code:`encoder_counter.cpp`

* Compile the file from command line using the command:

.. code-block:: bash

  g++ -std=c++11 -o encoder_counter encoder_counter.cpp -lpthread

* Run the executable that is generated:

.. code-block:: bash

  ./encoder_counter

* The following (or similar) will be printed to the terminal:

.. code-block:: bash

  counts = [1, 1, 1, 1]
  counts = [10, 10, 10, 10]
  counts = [20, 20, 20, 20]
  counts = [30, 30, 30, 30]
  counts = [40, 40, 40, 40]
  ...



Monitoring multiple GPIO pins for the encoder channels
******************************************************

In order to monitor events on multiple GPIO pins, we use :ref:`comm-protocol-GPIO-gpiod-library`. Documentation of the the C++ bindings for the gpiod library can be found at the following two locations:

  * `gpiod documentation hosted by dlang <https://libgpiod-dlang.dpldocs.info/gpiod.html>`_
  * `gpiod doxygen documentation hosted by lane-fu <https://www.lane-fu.com/linuxmirror/libgpiod/doc/html/index.html>`_

Piece together the following to reimplement the :code:`threadForCounting()` function to monitor the actual encoder channel and count all events that are detected:

| **Step 1**
| Specify the chip name and line numbers where the encoders channels are connected to the computer:

  .. code-block:: cpp

    // Function implementation
    void threadForCounting()
    {
      // Specify the gpio chip name of the GPIO interface
      const char * gpio_chip_name = "/dev/gpiochip0";

      // Specify the line numbers where the encoder channels are connected
      int line_number_for_m1a = 105;
      int line_number_for_m1b = 106;
      int line_number_for_m2a =  84;
      int line_number_for_m2b = 130;

  .. note::

    See the information for the :ref:`single board computer <single-board-computers>` that you are using to get some guidance on how to obtain the chip name and line numbers.

    The values above are for an Nvidia Jetson Xavier NX running Jetpack 5.x with the encoder channels connected to physical pin numbers {29, 31, 33, 35} on the 40-pin header.


  | **Step 2**
  | Initialise the :code:`gpiod` libary objects required for monitoring multiple line simultaneously, i.e., a :code:`bulk` of lines.

  .. code-block:: cpp

      // Initialise a GPIO chip, line, and event objects
      struct gpiod_chip *chip;
      struct gpiod_line *line_m1a;
      struct gpiod_line *line_m1b;
      struct gpiod_line *line_m2a;
      struct gpiod_line *line_m2b;
      struct gpiod_line_bulk line_bulk;
      struct gpiod_line_event event;
      struct gpiod_line_bulk event_bulk;


  | **Step 3**
  | Specify the timeout that is used as the wait time for monitoring, and initialise variables for various flags.

  .. code-block:: cpp

      // Specify the timeout specifications
      // > The first entry is seconds
      // > The second entry is nano-seconds
      struct timespec timeout_spec = { 0, 10000000 };
      
      // Intialise a variable for the flags returned by GPIO calls
      int returned_wait_flag;
      int returned_read_flag;


  | **Step 4**
  | Open the GPIO chip and lines, and put the lines together into the :code:`line_bulk` object.

  .. code-block:: cpp

      // Open the GPIO chip
      chip = gpiod_chip_open(gpio_chip_name);
      // Retrieve the GPIO lines
      line_m1a  = gpiod_chip_get_line(chip,line_number_for_m1a);
      line_m1b  = gpiod_chip_get_line(chip,line_number_for_m1b);
      line_m2a = gpiod_chip_get_line(chip,line_number_for_m2a);
      line_m2b = gpiod_chip_get_line(chip,line_number_for_m2b);
      // Initialise the line bulk
      gpiod_line_bulk_init(&line_bulk);
      // Add the lines to the line bulk
      gpiod_line_bulk_add(&line_bulk, line_m1a);
      gpiod_line_bulk_add(&line_bulk, line_m1b);
      gpiod_line_bulk_add(&line_bulk, line_m2a);
      gpiod_line_bulk_add(&line_bulk, line_m2b);

      // Display the status
      std::cout << "[ENCODER COUNTER] Chip " << gpio_chip_name << " opened and lines " << line_number_for_m1a << ", " << line_number_for_m1b << ", " << line_number_for_m2a << " and " << line_number_for_m2a << " retrieved";


  | **Step 5**
  | Specify the type of line event that is to be monitored. The three options of rising edge, falling edge, or both are all shown, only one should be uncommented.

  .. code-block:: cpp

      // Request the line events to be monitored
      // > Note: only one of these should be uncommented
      gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
      //gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
      //gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");


  | **Step 6**
  | Enter the while loop for monitoring the encoder channels while the flag indicates to do so.

  .. code-block:: cpp

      // Enter a loop that monitors the encoders
      while (m_threads_should_count)
      {

  | **Step 7**
  | Call the :code:`gpiod` library function that monitors for events on the all on the lines in the line bulk. This function stalls the execution here until either: an event is detected; or the timeout duration elapses.

  .. code-block:: cpp

        // Monitor for the requested events on the GPIO line bulk
        // > Note: the function "gpiod_line_event_wait" returns:
        //    0  if wait timed out
        //   -1  if an error occurred
        //    1  if an event occurred.
        returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);


  | **Step 8**
  | Lock the mutex and process any events that occurred, incrementing the respective count variable for each event on a line.

  .. code-block:: cpp

        // Lock the mutex before processing the events
        m_counting_mutex.lock();

        // Respond based on the the return flag
        if (returned_wait_flag == 1)
        {
          // Get the number of events that occurred
          int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

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
              if (this_line_number == line_number_for_m1a)
                m_counts_for_m1a++;
              else if (this_line_number == line_number_for_m1b)
                m_counts_for_m1b++;
              else if (this_line_number == line_number_for_m2a)
                m_counts_for_m2a++;
              else if (this_line_number == line_number_for_m2b)
                m_counts_for_m2b++;

            } // END OF: "if (returned_read_flag == 0)"

          } // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

        } // END OF: "if (returned_wait_flag == 1)"

        // Get a local copy of the flag
        bool should_count_local_copy = m_threads_should_count;

        // Unlock the mutex
        m_counting_mutex.unlock();


  | **Step 9**
  | Break the monitoring loop if the flag indicates to do so, and release the GPIO lines so that other processes can access these GPIO lines if they need to.

  .. code-block:: cpp

        // Break if necessary
        if (!should_count_local_copy)
          break;

      } // END OF: "while (true)"

      // Release the lines
      gpiod_line_release_bulk(&line_bulk);
      // Close the GPIO chip
      gpiod_chip_close(chip);
      // Inform the user
      std::cout << "[ENCODER COUNTER] Lines released and GPIO chip closed";

    } // END OF: "void threadForCounting()"



Reading and displaying the current encoder counts (and resetting)
*****************************************************************

The :code:`threadForReading()` function in :ref:`the boiler plate code above <building-block-gpio-encoder-counting-boiler-plate>` already makes a thread-safe copy of the counts variables and displays those value. Hence there is nothing more that need to be implemented for this step.

That said, such encoders are often referred to as "incremental" encoders. Hence we treat the :code:`threadForCounting()` function as being responsible for keeping track of the increment in encoder event since that the previous read of the counts variables. This shifts the responsibility to the :code:`threadForReading()` function for performing the desired computations with the counts, for example, keeping track of the cumulative counts or performing wheel odometry computations. The justification is that:

* The :code:`threadForCounting()` function runs at a high frequency (i.e., the variable frequency of encoder events), hence we should minimise computations performed by this thread.

* The :code:`threadForReading()` functions runs at a lower frequency (i.e., the fixed frequency that we specify), hence it is more flexible to implement processing of the encoder counts in this thread.

| Edit the :code:`threadForReading()` function as follows to track cumulative counts and reset the shared variable to zero with each read:
|  a) Declare :code:`static` variables for the cumulative counts. 
|  b) Enter a while loop that continues while the flag indicates to do so.
|  c) Lock the mutex, then take a local copy the shared variables, **then reset the shared counting variables to zero**, then unlock the mutex.
|  d) Add the local copy of the counts to the cumulative total.
|  e) Display the cumulative counts.
|  f) Sleep for a short time so that the while loop proceed at an approximate fixed frequency.

  .. code-block:: cpp

    // Function implementation
    void threadForReading()
    {
      static int cumulative_counts_for_m1 = 0;
      static int cumulative_counts_for_m2 = 0;

      while(m_threads_should_count)
      {
        // Get a local copy of the variables
        m_counting_mutex.lock();
        int counts_for_m1a_local_copy = m_counts_for_m1a;
        int counts_for_m1b_local_copy = m_counts_for_m1b;
        int counts_for_m2a_local_copy = m_counts_for_m2a;
        int counts_for_m2b_local_copy = m_counts_for_m2b;
        bool should_count_local_copy  = m_threads_should_count;
        // Reset the shared counting variables to zero
        m_counts_for_m1a = 0;
        m_counts_for_m1b = 0;
        m_counts_for_m2a = 0;
        m_counts_for_m2b = 0;
        m_counting_mutex.unlock();

        // Add the counts to the cumulative total
        cumulative_counts_for_m1 += (counts_for_m1a_local_copy + counts_for_m1b_local_copy);
        cumulative_counts_for_m2 += (counts_for_m2a_local_copy + counts_for_m2b_local_copy);

        // Display the cumulative counts
        printf("cumulative counts = [%d, %d]\n", cumulative_counts_for_m1, cumulative_counts_for_m2);

        // Break if necessary
        if (!should_count_local_copy)
          break;

        // Sleep for a bit
        usleep(100000);
      }
    }


Compile and run with the actual encoders 
****************************************

Add the :code:`gpiod` library included to the top of your file:

.. code-block::

  #include <gpiod.h>

Compile and run the file from command line, this time adding the compile flag for the :code:`gpiod` library, i.e.:

.. code-block:: bash

  g++ -std=c++11 -o encoder_counter encoder_counter.cpp -lpthread -lgpiod
  ./encoder_counter

For the few seconds that the code is counting, inject encoder pulses into the respective pins.
  


.. _building-block-gpio-encoder-counting-put-into-ros:

Putting it into a ROS node
**************************

Putting the stand-alone C++ developed above into a ROS node is as simple as:

* Start with a :ref:`ROS node that spins at a fixed frequency <ros-code-spin-at-frequency>`.

* Add the code of the main function in stand-alone C++ code to the main function of the ROS node.

  * Remove the :code:`sleep(3);`.
  * The :code:`ros::spin()` performs the function of keeping the main function alive, and hence the encoder counting is on going until the node is shutdown.
  * Hence, main function code before :code:`sleep(3);` goes before :code:`ros::spin()`, and code after :code:`sleep(3);` goes after :code:`ros::spin()`

* Add the code of the :code:`threadForReading` function in stand-alone C++ code to the :code:`timerCallback(...)` function of the ROS node.

  * Remove the displaying of the encoder counts and instead publish the encoder counts and instead display that data for other nodes to process.

* Add the ROS node cpp file to the :code:`CMakeList.txt` as per the :ref:`ros-code-cmakelists` page.
  * You need to add the :code:`-lgpiod` flag at the end of the :code:`target_link_libraries(...)` instruction, for example:

  .. code-block::

    target_link_libraries(encoder_counter ${catkin_LIBRARIES} -lgpiod)

* You can view an example implementation of such a ROS node in the `asclinic-system respository <https://gitlab.unimelb.edu.au/asclinic/asclinic-system>`_ at the location:

  .. code-block:: bash

    catkin_ws/src/asclinic_pkg/src/nodes/

  in the file :code:`encoder_read_multi_threaded.cpp`.


Launch file with parameters 
***************************

The line numbers of the GPIO pins to which the encoder channels are connected are likely to change depending on the situation, for example:

* Due to an operating system update.
* Due to running the code on different computers such as an Nvidia Jetson or a Raspberry Pi.

Hence, the line numbers are perfect candidates to input are parameters in a launch file. This allows launch the same node on different systems using the same C++ code.

Add the parameters to the launch file as follows:

.. code-block:: bash

  <launch>
    <group ns="asc">
      <!-- LAUNCH AN ENCODER COUNTER NODE -->
      <node
        pkg    = "asclinic_pkg"
        name   = "encoder_counter"
        output = "screen"
        type   = "encoder_counter"
        >
        <param
          name   = "line_number_for_m1a"
          value  = "105"
        />
        <param
          name   = "line_number_for_m1b"
          value  = "106"
        />
        <param
          name   = "line_number_for_m2a"
          value  = "84"
        />
        <param
          name   = "line_number_for_m2b"
          value  = "130"
        />
      </node>
    </group>
  </launch>

For each parameter, add the following to the main function of the ROS node to get the parameter variable into the respective member variable:

.. code-block:: cpp

  // > For motor 1 channel A
  if ( !nodeHandle.getParam("line_number_for_m1a", m_line_number_for_m1a) )
  {
    // Display an error message
    ROS_INFO("[ENCODER COUNTER] FAILED to get \"line_number_for_m1a\" parameter. Using default value instead.");
  }

.. note::

  Two nodes can**NOT** access the same GPIO line. The first node to open a line blocks other processes from accessing that line.



|

----

.. image:: https://i.creativecommons.org/l/by/4.0/88x31.png
  :alt: Creative Commons License
  :align: left
  :target: http://creativecommons.org/licenses/by/4.0/

| Paul N. Beuchat, 2023
| This page is licensed under a `Creative Commons Attribution 4.0 International License <http://creativecommons.org/licenses/by/4.0/>`_.

----

|
