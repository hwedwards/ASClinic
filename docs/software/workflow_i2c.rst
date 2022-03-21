.. _workflow-i2c:

ROS Interface with an I2C Bus
=============================

Once the device is physically connected to an I2C bus, and before attempting to interface using ROS, you should use the command line tool to check the device is properly connected.
See the :ref:`I2C page <comm-protocol-i2c>` for details of the command line tools, and see the :ref:`SBC page <single-board-computers>` for details of the I2C buses for the relevant SBC.


For example, if the device is connected to the I2C bus of pins (3,5) of the J21 expansion header of the Jetson Xavier NX, then this corresponds to I2C bus 8. Hence the addresses of the connected devices can be quickly checked with the command:

.. code-block:: bash

	sudo i2cdetect -y -r 8

To interface with I2C devices via a ROS node, there are two templates provided:

* The file named :code:`template_i2c_external.cpp`: this template manages the I2C bus for devices connected to the exterior of the robot. In the default build wiring configuration, this is I2C bus 8 of the Jetson Xavier NX.
* The file named :code:`template_i2c_internal.cpp`: this template manages the I2C bus for devices connected in the interior of the robot. In the default build wiring configuration, this is I2C bus 1 of the Jetson Xavier NX.

Both files are located in the repository at the relative file path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/nodes/

Both files are extensively commented and the comments serve as the documentation for how to edit the file to implement your use case.
The I2C templates are C++ files because the library used for interfacing with the I2C buses is C++ based, and hence the driver developed for the I2C devices are also written in C++.

Both template nodes are launched by file :code:`template_i2c.launch`.
You should review the launch file to check how it is structured.
Then you can launch the template I2C nodes with:

.. code-block:: bash

  roslaunch asclinic_pkg template_i2c.launch


..
	You should review the launch file to check how it is structured to add a parameter to each node upon launch (see also the note below).


..
	.. note::
	  The I2C bus to manage is specified as a parameter in the launch file.
	  Hence, to change the bus number being managed, you simply need to change the line number parameter in the launch file and re-launch the node.

	  * This has the benefit that you can change the bus number without needing to recompile the code.
	  * This has the disadvantage that you cannot specify the line number parameter when using :code:`rosrun`.



.. note::
  When you copy either template C++ file, you will need to add it to the :code:`CMakeLists.txt` file in the repository at the relative file path:

  .. code-block:: bash

    catkin_ws/src/asclinic_pkg/CMakeLists.txt

  Simply duplicate and accordingly edit the lines where the name of the respective template I2C C++ file appears.
  Most important is that the :code:`add_executable(...)` needs to list the executable files for any I2C driver that in included by the node. For example, the :code:`template_i2c_external` node has the following :code:`add_executable(...)` in the :code:`CMakeLists.txt` file:

  .. code-block:: bash

    add_executable(template_i2c_external
      src/nodes/template_i2c_external.cpp
      src/drivers/src/i2c_driver/i2c_driver.cpp
      src/drivers/src/vl53l1x/vl53l1x.cpp
      src/drivers/src/vl53l1x/core/VL53L1X_api.c
      src/drivers/src/vl53l1x/platform/vl53l1_platform.c
      )


.. note::

  You can view the source code of the available drivers at the relative path:

  .. code-block:: bash

    catkin_ws/src/asclinic_pkg/src/drivers/src/


Using the servo driver interface
********************************

The servos for the robot are driven by a `16-channel 12-bit PWM/servo driver with I2C interface <https://www.adafruit.com/product/815>`_, which is a breakout board for the `PCA9685 chip <https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/ic-led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685>`_ (`datasheet available here <https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf>`_). A C++ driver for interfacing with this breakout board over the I2C interface is included in this repository at the relative path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/drivers/src/pca9685/

The :code:`template_i2c_internal.cpp` is setup to use this PCA9685 driver, and the following steps detail how you can add this functionality to your own ROS C++ I2C node.


**Step 1.** Include the PCA9685 header in your I2C node by adding the following:

  .. code-block:: cpp

    #include "pca9685/pca9685.h"


**Step 2.** Instantiate a :code:`PCA9685` object as a member variable for your node so that it can be accessed from all functions within the node. To do this, add the following lines of code just after where you instantiate the :code:`I2C_Driver` object:

  .. code-block:: cpp

    // > For the PCA9685 PWM Servo Driver driver
    const uint8_t m_pca9685_address = 0x42;
    PCA9685 m_pca9685_servo_driver (&m_i2c_driver, m_pca9685_address);

  Where the :code:`m_i2c_driver` variable is the variable of type :code:`I2C_Driver` that should already be in your code.

  .. note::

    The default I2C address of the PCA9685 chip is :code:`0x40`, however, this is also the I2C address of the INA260 current sensor. The I2C address of the PCA9685 is hardware selectable in the range :code:`0x40` to :code:`0x4F` by soldering pads on the breakout board.


**Step 3.** Initialise the PCA9685 chip and set the frequency of its output channels by adding the following code to the main function of your I2C node.

  .. code-block:: cpp

    // SET THE CONFIGURATION OF THE SERVO DRIVER

    // Specify the frequency of the servo driver
    float new_frequency_in_hz = 50.0;

    // Call the Servo Driver initialisation function
    bool verbose_display_for_servo_driver_init = false;
    bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);

    // Display if an error occurred
    if (!result_servo_init)
    {
      ROS_INFO_STREAM("FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
    }

  .. note::

    It is generally recommended to use a 50 Hz signal for commanding servos, hence the frequency of the PCA9685 output channels is set to 50 Hz by this code snippet above.


Servos are typically commanded by providing them with a PWM input signal that has a pulse width ranging from 1000 microseconds to 2000 mircoseconds, with a pulse width of 1500 microseconds commands the centre position of the servo (or no rotation for a continuous rotation servo). As the PCA9685 has 16 output channels (i.e., it can command 16 separate servos), it is convenient to have a ROS message type that specifies the channel number and desired pulse width.

**Step 4.**  If it does not already exist, then define a :code:`ServoPulseWidth` message type by adding a file named :code:`:code:`ServoPulseWidth.msg` to the relative path:

  .. code-block:: bash

    catkin_ws/src/asclinic_pkg/msg/

  And write the following for the contents of the file:

  .. code-block:: bash

    uint16 channel
    uint16 pulse_width_in_microseconds


**Step 5.** Include the :code:`ServoPulseWidth` message type in your I2C node by adding the following with the other includes of your I2C node:

  .. code-block:: cpp

    // Include the asclinic message types
    #include "asclinic_pkg/ServoPulseWidth.h"

    // Namespacing the package
    using namespace asclinic_pkg;


At this stage, if you try to compile your I2C node with :code:`catkin_make`, it will likely fail because the headers :code:`ServoPulseWidth.h` and :code:`pca9685.h` header are not found. The :code:`CMakeLists.txt` needs to be adjusted to give the required compilation directives.

**Step 6.** Adjust the :code:`CMakeLists.txt` to add the :code:`ServoPulseWidth.msg` to the following part:

  .. code-block:: bash

    add_message_files(
      FILES
      TemplateMessage.msg
      ServoPulseWidth.msg
    )


**Step 7.** Adjust the :code:`CMakeLists.txt` to add the :code:`pca9685.cpp` as an executable to your I2C node, i.e., in a form similar to the following:

  .. code-block:: bash

    add_executable(template_i2c_internal    src/nodes/template_i2c_internal.cpp
                                            src/drivers/src/i2c_driver/i2c_driver.cpp
                                            src/drivers/src/pololu_smc_g2/pololu_smc_g2.cpp
                                            src/drivers/src/pca9685/pca9685.cpp)


**Step 8.** Compile your I2C node with :code:`catkin_make` to check that the above steps are correctly implemented.

  .. note::

    Ensure that you have the latest version of the PCA9685 driver from the repository, i.e., ensure the the contents of your repository at the relative path:

    .. code-block:: bash

      catkin_ws/src/asclinic_pkg/src/drivers/src/pca9685/

    is up to date with the contents of the `same directory in the main repository <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/tree/master/catkin_ws/src/asclinic_pkg/src/drivers/src/pca9685>`__.


In order actually command your servo, you will need to create a subscriber for sending commands to your I2C node, and then in the subscriber callback send those commands over the I2C interface.


**Step 9.** Add a subscriber to the :code:`main` function of your I2C node for responding to requests to command the servo:

  .. code-block:: cpp

    ros::Subscriber set_servo_pulse_width_subscriber = nodeHandle.subscribe("set_servo_pulse_width", 1, templateSubscriberCallback);

  .. note::

    Be sure to change the :code:`nodeHandle` to be appropriate for the namespace within which you want this topic to operate.


**Step 10.** Add the subscriber callback to your I2C node. The subscriber callback should read the channel and requested pulse width from the :code:`ServoPulseWidth` type message received, and send the command to the PCA9685 over I2C using the function :code:`set_pwm_pulse_in_microseconds` that is provided by the PCA9685 driver.

  .. code-block:: cpp

    void templateServoSubscriberCallback(const ServoPulseWidth& msg)
    {
      // Extract the channel and pulse width from the message
      uint8_t channel = msg.channel;
      uint16_t pulse_width_in_us = msg.pulse_width_in_microseconds;

      // Display the message received
      ROS_INFO_STREAM("Message receieved for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

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
        ROS_INFO_STREAM("FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
      }
    }


  .. warning::

    It is possible to damage a servo by sending a pulse width command that is too large of too small (i.e., outside the range 1000-2000 microseconds). The pulse width essentially specifies the position to which the servo should move. If the pulse width specifies a position beyond the physically possible range, then the motor within the servo will still try to drive to that unreachable position, and the most likely part of fail is the that gears inside the servo break.

    If you are unfamiliar with how servos work, then you should read through both of these tutorials:

      * `Sparkfun servos explained <https://www.sparkfun.com/servos>`_
      * `Sparkfun hobby servo tutorial <https://learn.sparkfun.com/tutorials/hobby-servo-tutorial/all>`_


**Step 11.** Connect a servo to your robot, compile your I2C node with :code:`catkin_make`, launch your I2C node, and send a message to command the position of the servo. You can send a message from command line as follows:

  .. code-block:: bash

    rostopic pub /<namespace_of_your_topic>/set_servo_pulse_width asclinic_pkg/ServoPulseWidth "{channel: 15, pulse_width_in_microseconds: 1100}"

  Where :code:`<namespace_of_your_topic>` is set appropriate to the :code:`nodeHandle` you used when subscribing to the topic in Step 9 above. When this message is recieved by your I2C node, the callback function of Step 10 above will set channel 15 of the servo driver breakout board to have a pulse width of 1100 microseconds.


You can `view an example of these steps implemented <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/blob/master/catkin_ws/src/asclinic_pkg/src/nodes/template_i2c_internal.cpp>`__ in the :code:`template_i2c_internal.cpp` file of the main repository.




Using the ToF distance sensor
*****************************

The VL53L1X time-of-flight (ToF) distance sensor (see the :ref:`bill of materials<bom>` for further links to the product and datasheet) offers a quick and easy interface for distance measurements in the range of 30 centimeters to 4 meters, and can be a rich source of information about the environment when using multiple ToF sensors simultaneously. The VL53L1X has a fixed I2C address that cannot be changed in hardware, and changing the I2C address in software requires one GPIO pin per sensor and the change does not persist (i.e., it must be changed on every startup/reset). A C++ driver for interfacing with one or multiple of these breakout boards over the I2C interface is included in this repository at the relative path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/drivers/src/vl53l1x/

The :code:`template_i2c_external.cpp` is setup to use this VL53L1X driver, and the following steps detail how you can add this functionality to your own ROS C++ I2C node.


**Step 1.** Include the VL53L1X header in your I2C node by adding the following:

  .. code-block:: cpp

    #include "vl53l1x/vl53l1x.h"


**Step 2.** Instantiate a :code:`VL53L1X` object as a local variable within the :code:`main` function of your node. If you are using just one VL53L1X distance sensor, then add the following lines of code somewhere after you instantiate the :code:`I2C_Driver` object:

  .. code-block:: cpp

    // Initialise an object for the VL53L1X distance sensor
    // > If connected directly to the I2C bus
    const uint8_t vl53l1x_address = 0x29;
    VL53L1X vl53l1x_object (&i2c_driver, vl53l1x_address);

  Where the :code:`i2c_driver` variable is the variable of type :code:`I2C_Driver` that should already be in your code. If you are using multiple VL53L1X distance sensors, then add the following lines of code somewhere after you instantiate the :code:`I2C_Driver` object:

  .. code-block:: cpp

    // Initialise an object for the VL53L1X distance sensor
    // > If connected to the I2C multiplexer
    const uint8_t vl53l1x_address = 0x29;
    const uint8_t mux_channel     = 0;
    const uint8_t mux_i2c_address = 0x70;
    VL53L1X vl53l1x_object_on_mux_ch0 (&i2c_driver, vl53l1x_address, mux_channel, mux_i2c_address);


  .. note::

    The default I2C address of the VL53L1X chip is :code:`0x29`, and as this cannot be changed in hardware. You should always use this address when instantiating a :code:`VL53L1X` object.

  .. note::

    The default I2C address of the TCA9548A multiplexer chip is :code:`0x70`, however, this is also an I2C address of the PCA9685 servo driver that is enabled at power-up of the PCA9685 and hence should not be used by other devices on the same bus. The I2C address of the TCA9548A multiplexer is hardware selectable in the range :code:`0x70` to :code:`0x77` by soldering pads on the breakout board. As the default configuration of the robot has the PCA9685 chip on the internal I2C bus and the VL53L1X chips on the external I2C bus, it is fine to use the default address of :code:`0x70` for the TCA9548A multiplexer chip.

  .. note::..

    If you need to access the :code:`VL53L1X` object from other functions within the node (for example, only take distance measurements when a subscriber callback is triggered), then declare the :code:`VL53L1X` object as a member variable where your I2C driver is also declared as a member variable.


**Step 3.** Initialise the VL53L1X chip, set the distance mode, and start repeated measurements of the VL53L1X chip (called "ranging") by adding the following code to the main function of your I2C node.

  .. code-block:: cpp

    // Specify the distancing specifications
    // > Distance Mode (1 = short distance, 2 = long distance)
    uint16_t distance_mode = 2;

    // Initialise the VL53L1X distance sensor
    bool is_available_vl53l1x = vl53l1x_object.initialise_and_start_ranging(distance_mode);

    if (!is_available_vl53l1x)
    {
      ROS_INFO("FAILED - while initialising the VL53L1X distance sensor. Sensor is NOT available for usage.");
    }


  .. note::

    Taken directly from the documentation:

    *Long distance mode allows the longest possible ranging distance of 4 m to be reached. However, this maximum ranging distance is impacted by ambient light.*

    *Short distance mode is more immune to ambient light, but its maximum ranging distance is typically limited to 1.3 m.*


At this stage, if you try to compile your I2C node with :code:`catkin_make`, it will likely fail because the :code:`vl53l1x.h` header is not found. The :code:`CMakeLists.txt` needs to be adjusted to give the required compilation directives.


**Step 4.** Adjust the :code:`CMakeLists.txt` to add the :code:`vl53l1x.cpp` as an executable to your I2C node as well as the :code:`.c` driver provide by the manufacturer of the VL53L1X chip, i.e., in a form similar to the following:

  .. code-block:: bash

    add_executable(template_i2c_external      src/nodes/template_i2c_external.cpp
                                              src/drivers/src/i2c_driver/i2c_driver.cpp
                                              src/drivers/src/vl53l1x/vl53l1x.cpp
                                              src/drivers/src/vl53l1x/core/VL53L1X_api.c
                                              src/drivers/src/vl53l1x/platform/vl53l1_platform.c)



**Step 5.** Compile your I2C node with :code:`catkin_make` to check that the above steps are correctly implemented.

  .. note::

    Ensure that you have the latest version of the VL53L1X driver from the repository, i.e., ensure the the contents of your repository at the relative path:

    .. code-block:: bash

      catkin_ws/src/asclinic_pkg/src/drivers/src/vl53l1x/

    is up to date with the contents of the `same directory in the main repository <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/tree/master/catkin_ws/src/asclinic_pkg/src/drivers/src/vl53l1x>`__.


In order make the distance measurements available to your other nodes, you will need to create a pulisher for sending messages from your I2C node.


**Step 6.** Add a publisher to the :code:`main` function of your I2C node for publishing messages with the measured distance:

  .. code-block:: cpp

    ros::Publisher tof_distance_publisher = nodeHandle.advertise<std_msgs::UInt16>("tof_distance", 10, false);

  .. note::

    Be sure to change the :code:`nodeHandle` to be appropriate for the namespace within which you want this topic to operate.


**Step 7.** Add the following within a :code:`while (ros::ok())` loop of your main function in order to read the distance measurements over I2C.

  .. code-block:: cpp

    // Initialise a variable with loop rate for
    // polling the sensors
    // > Input argument is the frequency in hertz, as a double
    ros::Rate loop_rate(0.75);

    // Enter a loop that continues while ROS is still running
    while (ros::ok())
    {
      // Read data from the VL53L1X distance sensor
      VL53L1X_Result_t tof_res;
      bool success_get_distance = vl53l1x_object.get_distance_measurement(&tof_res);

      // If a result was succefully retrieved:
      if (success_get_distance)
      {
        // If the result status is good:
        if (tof_res.Status == 0)
        {
          // Then publish the distance measured
          std_msgs::UInt16 msg;
          msg.data = tof_res.Distance;
          tof_distance_publisher.publish(msg);
        }
        else
        {
          // Otherwise display the error status
          uint16_t temp_status = tof_res.Status;
          ROS_INFO_STREAM("FAILED - VL53L1X \"get_distance_measurement\" returned with an error status, status = " << temp_status << ", distance = " << tof_res.Distance << ", ambient = " << tof_res.Ambient << ", signal per SPAD = " << tof_res.SigPerSPAD << ", # of SPADs = " << tof_res.NumSPADs);
        }
      }
      else
      {
        // Otherwise display the error
        ROS_INFO("FAILED - to \"get_distance_measurement\" from VL53L1X distance sensor.");
      }


      // Spin once so that this node can service any
      // callbacks that this node has queued.
      ros::spinOnce();

      // Sleep for the specified loop rate
      loop_rate.sleep();
    } // END OF: "while (ros::ok())"


  .. note::

    It is likely that your I2C node already has a :code:`while (ros::ok())` loop, and hence from the code snippet above you only need to add the parts that relate to reading and publishing/displaying the distance measurement.

  .. note::

    If you have multiple VL53L1X distance sensors connected via a TCA9548A multiplexer, then you will have one :code:`VL53L1X` object in your code for each sensor (for example, with variable names: :code:`vl53l1x_object_on_mux_ch0`, :code:`vl53l1x_object_on_mux_ch1`, etc.). You will need to call the :code:`.get_distance_measurement` function separately for each such object, for example:

    .. code-block:: cpp

      VL53L1X_Result_t tof_res_ch0;
      bool success_get_distance_ch0 = vl53l1x_object_on_mux_ch0.get_distance_measurement(&tof_res_ch0);

      VL53L1X_Result_t tof_res_ch1;
      bool success_get_distance_ch1 = vl53l1x_object_on_mux_ch1.get_distance_measurement(&tof_res_ch1);

    Once you have more than two distance sensors connected, it is advisable to invest time into writing functions, structures, and messages types that read and send out multiple distances in a more flexible and encapsulated fashion.


  .. warning::

    Lighting conditions can affect the measurements taken by the VL53L1X sensor, and can cause the result flag to indicate an error. The example above is quite simple in that is accepts measurements with a "good" status, and prints out the details of a measurement with any other status. It is advised that you conduct tests with various lighting conditions that mimic the potential use cases of your robot, and that you investigate any unexpected behaviour.

    To assist with interrogating the measurements further, the result struct is defined in :code:`VL53L1X_api.h` as follows:

    .. code-block::

      typedef struct {
          uint8_t     Status;        /*!< ResultStatus */
          uint16_t    Distance;      /*!< ResultDistance */
          uint16_t    Ambient;       /*!< ResultAmbient */
          uint16_t    SigPerSPAD;    /*!< ResultSignalPerSPAD */
          uint16_t    NumSPADs;      /*!< ResultNumSPADs */
      } VL53L1X_Result_t;

    Where :code:`SPAD` stands for single photon avalanche diode, of which the VL53L1X sensor has an array of 16x16 SPADs.



**Step 8.** Connect a VL53L1X distance sensor to your robot, compile your I2C node with :code:`catkin_make`, launch your I2C node, and listen to the messages published on the respective topic. You can :code:`echo` messages on a topic from command line as follows:

  .. code-block:: bash

    rostopic echo /<namespace_of_your_topic>/tof_distance

  Where :code:`<namespace_of_your_topic>` is set appropriate to the :code:`nodeHandle` you used when advertising the topic in Step 6 above.


You can `view an example of these steps implemented <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/blob/master/catkin_ws/src/asclinic_pkg/src/nodes/template_i2c_external.cpp>`__ in the :code:`template_i2c_external.cpp` file of the main repository.
