.. _building-block-i2c-pololu-smc-g2:

Using the DC motor driver interface (Pololu SMC G2)
***************************************************

The DC motors for the main drive wheel of the robot are driven by a `Pololu Simple Motor Controller (SMC) G2 <https://www.pololu.com/product/1367>`. A C++ driver for interfacing with this board over the I2C interface is included in this repository at the relative path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/drivers/src/pololu_smc_g2/

The :code:`i2c_motors_and_servos.cpp` is setup to use this Pololu SMC G2 driver, and the following steps detail how you can add this functionality to your own ROS C++ I2C node.



.. note::

  The "controller" in the name "Simple Motor Controller" should be understood correctly.

  **What it is:** The Pololu SMC G2 controls the duty cycle of the pulse width modulation (PWM) signal that power the motor based on the duty cycle that is requested via an I2C command. Other interfaces are possible for requesting the duty cycle (USB, TTL serial, analog voltage (potentiometer), and hobby radio control (RC)) but are not described in this wiki.

  What it is **NOT**: The Pololu SMC G2 does **NOT** control the rotational speed of the motor. The board has **NO options** for receiving feedback of the motors velocity (e.g., via an encoder) and controlling the rotational velocity based on the feedback.



**Step 1.** Include the Pololu SMC G2 header in your I2C node by adding the following:

  .. code-block:: cpp

    #include "pololu_smc_g2/pololu_smc_g2.h"


**Step 2.** Instantiate two :code:`Pololu_SMC_G2` objects as member variables for your node so that it can be accessed from all functions within the node; one is for the left wheel and the other for the right wheel. To do this, locate the lines of code where you instantiate the :code:`I2C_Driver` object, which should something like this:

  .. code-block:: cpp

    // MEMBER VARIABLES FOR THIS NODE:
    // > For the I2C driver
    const char * m_i2c_device_name = "/dev/i2c-1";
    I2C_Driver m_i2c_driver (m_i2c_device_name);

and add the following lines of code just after:

  .. code-block:: cpp

    // > For the Pololu Simple Motor Controller (SMC) driver
    const uint8_t m_pololu_smc_address_left = 13;
    Pololu_SMC_G2 m_pololu_smc_left (&m_i2c_driver, m_pololu_smc_address_left);

    const uint8_t m_pololu_smc_address_right = 14;
    Pololu_SMC_G2 m_pololu_smc_right (&m_i2c_driver, m_pololu_smc_address_right);

  Where the :code:`m_i2c_driver` variable is the variable of type :code:`I2C_Driver` that should already be in your code.

  .. important::

    The default I2C address of the Pololu SMC G2 board is :code:`13` (i.e., :code:`0x0d`). Hence to use two Pololu SMC G2 on the same I2C bus, you need to make the addresses different.

    The I2C address of the Pololu SMC G2 is software selectable in the range 1 to 127 by using the `windows-based software provided by Pololu <https://www.pololu.com/docs/0J77/3.1>`_.

    The code snippet in this step suggests using address 13 for the left wheel and address 14 for the right wheel.

  .. note::

    As stated on `this page of the user guide <https://www.pololu.com/docs/0J77/6>`_: I2C is disabled by default, so you must enable it by checking the "Enable I2C" checkbox in the "Input settings" tab of the Simple Motor Control Center G2.



**Step 3.** Initialise the Pololu SMC G2 board and set limits on:

  * The maximum current allowed
  * The maximum duty cycle allowed
  * The maximum change in duty cycle per update period increment of the board (referred to in teh documentation as acceleration for increase in duty cycle and deceleration for decreases in duty cycle).

by adding the following code to the main function of your I2C node.

  .. code-block:: cpp

    // SET THE CONFIGURATION OF EACH MOTOR CONTROLLER

    // Specify the various limits
    int new_current_limit_in_milliamps = 5000;
    int new_max_duty_cycle_limit = 2560;
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
      ROS_INFO_STREAM("Now initialising SMC with I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );

      // Call the Pololu SMC initialisation function
      bool verbose_display_for_SMC_init = false;
      bool result_smc_init = pololu_smc_pointer->initialise_with_limits(new_current_limit_in_milliamps,new_max_duty_cycle_limit,new_max_accel_limit,new_max_decel_limit,verbose_display_for_SMC_init);

      // Display if an error occurred
      if (!result_smc_init)
      {
        ROS_INFO_STREAM("FAILED - while initialising SMC with I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );
      }
    }

  .. note::

    The meaning of the limits and update period are described on the `Motor Setting page <https://www.pololu.com/docs/0J77/5.2>`_ of the Pololu SMC G2 documentation.

    The default update period of the board is 1 millisecond, hence 1000 Hz update rate.

    The base units that the board uses for duty cycle are a linear scale from 0 (for 0% duty cycle) to 3200 (for 100% duty cycle). These are the units used for the acceleration and deceleration limits of the board, hence, an acceleration limit of 1 means the duty cycle can increase by 1/32 of a percent per the update period (1ms by default).



**Step 4.**  If it does not already exist, then define a :code:`LeftRightFloat32` message type by adding a file named :code:`:code:`LeftRightFloat32.msg` to the relative path:

  .. code-block:: bash

    catkin_ws/src/asclinic_pkg/msg/

  And write the following for the contents of the file:

  .. code-block:: bash

    float32 left
    float32 right



**Step 5.** Include the :code:`LeftRightFloat32` message type in your I2C node by adding the following with the other includes of your I2C node:

  .. code-block:: cpp

    // Include the asclinic message types
    #include "asclinic_pkg/LeftRightFloat32.h"

    // Namespacing the package
    //using namespace asclinic_pkg;


At this stage, if you try to compile your I2C node with :code:`catkin_make`, it will likely fail because the headers :code:`LeftRightFloat32.h` and :code:`pololu_smc_g2.h` are not found. The :code:`CMakeLists.txt` needs to be adjusted to give the required compilation directives.

.. note::

  If you uncomment the line :code:`using namespace asclinic_pkg;`, then you can leave off the :code:`asclinic_pkg::` wherever it appears in the following steps.



**Step 6.** Adjust the :code:`CMakeLists.txt` to add the :code:`LeftRightFloat32.msg` to the following part:

  .. code-block:: bash

    add_message_files(
      FILES
      TemplateMessage.msg
      LeftRightFloat32.msg
    )



**Step 7.** Adjust the :code:`CMakeLists.txt` to add the :code:`pololu_smc_g2.cpp` as an executable to your I2C node, i.e., in a form similar to the following:

  .. code-block:: bash

    add_executable(i2c_motors_and_servos    src/nodes/i2c_motors_and_servos.cpp
                                            src/drivers/src/i2c_driver/i2c_driver.cpp
                                            src/drivers/src/pololu_smc_g2/pololu_smc_g2.cpp)



**Step 8.** Compile your I2C node with :code:`catkin_make` to check that the above steps are correctly implemented.

  .. note::

    Ensure that you have the latest version of the Pololu SMC G2 driver from the repository, i.e., ensure the the contents of your repository at the relative path:

    .. code-block:: bash

      catkin_ws/src/asclinic_pkg/src/drivers/src/pololu_smc_g2/

    is up to date with the contents of the `same directory in the main repository <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/tree/master/catkin_ws/src/asclinic_pkg/src/drivers/src/pololu_smc_g2>`__.


In order actually command your main drive wheels, you will need to create a subscriber for sending commands to your I2C node, and then in the subscriber callback send those commands over the I2C interface.



**Step 9.** Add a subscriber to the :code:`main` function of your I2C node for responding to requests to command the servo:

  .. code-block:: cpp

    // Initialise a subscriber for the duty cycle of the main drive motors
    ros::Subscriber set_motor_duty_cycle_subscriber = nodeHandle.subscribe("set_motor_duty_cycle", 1, driveMotorsSubscriberCallback);

  .. note::

    Be sure to change the :code:`nodeHandle` to be appropriate for the namespace within which you want this topic to operate.



**Step 10.** Add the subscriber callback to your I2C node. The subscriber callback should read the requested duty cycle for the left and right wheel from the :code:`LeftRightFloat32` type message received, and send the command to the Pololu SMC G2 over I2C using the function :code:`set_motor_target_speed_percent` that is provided by the Pololu SMC G2 driver.

  .. code-block:: cpp

    void driveMotorsSubscriberCallback(const asclinic_pkg::LeftRightFloat32& msg)
    {
      ROS_INFO_STREAM("Message received with left = " << msg.left << ", right = " << msg.right);

      // Clip the data to be in the range [-100.0,100.0]
      // > For the left value
      float pwm_duty_cycle_left = msg.left;
      if (pwm_duty_cycle_left < -100.0f)
        pwm_duty_cycle_left = -100.0f;
      if (pwm_duty_cycle_left > 100.0f)
        pwm_duty_cycle_left = 100.0f;
      // > For the right value
      float pwm_duty_cycle_right = msg.right;
      if (pwm_duty_cycle_right < -100.0f)
        pwm_duty_cycle_right = -100.0f;
      if (pwm_duty_cycle_right > 100.0f)
        pwm_duty_cycle_right = 100.0f;

      // Initialise one boolean variable for the result
      // of all calls to Pololu_SMC_G2 functions
      bool result;

      // SET THE TARGET DUTY CYCLE FOR EACH MOTOR CONTROLLER
      // > NOTE: it may be necessary to negate one or both
      //   of the duty cycles depending on the polarity
      //   with which each motor is plugged in

      // > Set the LEFT motor controller
      result = m_pololu_smc_left.set_motor_target_speed_percent(pwm_duty_cycle_left);
      if (!result)
        ROS_INFO_STREAM("FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << static_cast<int>(m_pololu_smc_left.get_i2c_address()) );

      // > Set the RIGHT motor controller
      result = m_pololu_smc_right.set_motor_target_speed_percent(-pwm_duty_cycle_right);
      if (!result)
        ROS_INFO_STREAM("FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << static_cast<int>(m_pololu_smc_right.get_i2c_address()) );
    }



**Step 11.** Compile your I2C node with :code:`catkin_make`, launch your I2C node, and send a message to command the duty cycle of the drive wheels. You can send a message from command line as follows:

  .. code-block:: bash

    rostopic pub --once <namespace_of_your_topic>/set_motor_duty_cycle asclinic_pkg/LeftRightFloat32 "{left: 10.1, right: 10.1}"

  Where :code:`<namespace_of_your_topic>` is set appropriate to the :code:`nodeHandle` you used when subscribing to the topic in Step 9 above. When this message is received by your I2C node, the callback function of Step 10 above will send the I2C command to the Pololu SMC G2 boards to set the duty cylce of the supply to each motor to be 10.1%.



**Step 12.** As the subscriber callback in Step 10 may change the duty cycle requested, it is beneficial to publish the duty cycle that was actually sent to the Pololu SMC G2 boards so that this information is available to any other node that needs it.

  Add a member variable for the publisher (where you instantiate the other member variables):

  .. code-block:: cpp

    // > Publisher for the current duty cycle
    //   setting of the main drive motors
    ros::Publisher m_current_motor_duty_cycle_publisher;

  Initialise the publisher in the :code:`main` function:

  .. code-block:: cpp

    // Initialise a publisher for the current duty cycle setting of the main drive motors
    m_current_motor_duty_cycle_publisher = nodeHandle.advertise<asclinic_pkg::LeftRightFloat32>("current_motor_duty_cycle", 10, true);

  Publish the current motor duty cycle just after they are set, i.e., at the end of the :code:`driveMotorsSubscriberCallback` function:

  .. code-block:: cpp

    // Publish the motor duty cycles
    asclinic_pkg::LeftRightFloat32 msg_current_duty_cycle;
    msg_current_duty_cycle.left  = pwm_duty_cycle_left;
    msg_current_duty_cycle.right = pwm_duty_cycle_right;
    m_current_motor_duty_cycle_publisher.publish(msg_current_duty_cycle);



You can `view an example of these steps implemented <https://gitlab.unimelb.edu.au/asclinic/asclinic-system/-/blob/master/catkin_ws/src/asclinic_pkg/src/nodes/i2c_motors_and_servos.cpp>`__ in the :code:`i2c_motors_and_servos.cpp` file of the main repository.
