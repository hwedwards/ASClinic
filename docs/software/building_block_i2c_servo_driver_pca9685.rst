.. _building-block-i2c-servo-driver-pca9685:

Using the servo driver interface (PCA9685)
******************************************

The servos for the robot are driven by a `16-channel 12-bit PWM/servo driver with I2C interface <https://www.adafruit.com/product/815>`_, which is a breakout board for the `PCA9685 chip <https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/ic-led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685>`_ (`datasheet available here <https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf>`_). A C++ driver for interfacing with this breakout board over the I2C interface is included in this repository at the relative path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/drivers/src/pca9685/

The :code:`i2c_for_sensors.cpp` is setup to use this PCA9685 driver, and the following steps detail how you can add this functionality to your own ROS C++ I2C node.


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

    add_executable(i2c_for_sensors    src/nodes/i2c_for_sensors.cpp
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

    ros::Subscriber set_servo_pulse_width_subscriber = nodeHandle.subscribe("set_servo_pulse_width", 1, servoSubscriberCallback);

  .. note::

    Be sure to change the :code:`nodeHandle` to be appropriate for the namespace within which you want this topic to operate.


**Step 10.** Add the subscriber callback to your I2C node. The subscriber callback should read the channel and requested pulse width from the :code:`ServoPulseWidth` type message received, and send the command to the PCA9685 over I2C using the function :code:`set_pwm_pulse_in_microseconds` that is provided by the PCA9685 driver.

  .. code-block:: cpp

    void servoSubscriberCallback(const asclinic_pkg::ServoPulseWidth& msg)
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


You can `view an example of these steps implemented <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/blob/master/catkin_ws/src/asclinic_pkg/src/nodes/i2c_for_sensors.cpp>`__ in the :code:`i2c_for_sensors.cpp` file of the main repository.



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
