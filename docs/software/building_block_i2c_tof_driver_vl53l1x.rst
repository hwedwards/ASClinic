.. _building-block-i2c-tof-driver-vl53l1x:

Using the ToF distance sensor (VL53L1X)
***************************************

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
