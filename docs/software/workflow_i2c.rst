.. _workflow-i2c:

ROS Interface with an I2C Bus
=============================

Checking the I2C bus
********************

Once the device is physically connected to an I2C bus, and before attempting to interface using ROS, you should use the command line tool to check the device is properly connected.
See the :ref:`I2C page <comm-protocol-i2c>` for details of the command line tools, and see the :ref:`single board computers (SBC) page <single-board-computers>` for details of the I2C buses for the relevant SBC.


For example, if the device is connected to the I2C bus of pins (3,5) of the J21 expansion header of the Jetson Xavier NX, then this corresponds to I2C bus 8. Hence the addresses of the connected devices can be quickly checked with the command:

.. code-block:: bash

	sudo i2cdetect -y -r 8



ROS templates
*************

To interface with I2C devices via a ROS node, there are two templates provided:

* The file named :code:`template_i2c_external.cpp`: this template manages the I2C bus for devices connected to the exterior of the robot. In the default build wiring configuration, this is I2C bus 8 of the Jetson Xavier NX.
* The file named :code:`template_i2c_internal.cpp`: this template manages the I2C bus for devices connected in the interior of the robot. In the default build wiring configuration, this is I2C bus 1 of the Jetson Xavier NX.

Both files are located in the repository at the relative file path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/nodes/

Both files are extensively commented and the comments serve as the documentation for how to edit the file to implement your use case.
The I2C templates are C++ files because the library used for interfacing with the I2C buses is C++ based, and hence the drivers developed for the I2C devices are also written in C++.

The two template nodes can be respectively launched by files :code:`i2c_internal.launch` and :code:`i2c_external.launch`.
You should review the launch files to check how they are structured.
Then you can launch the template I2C nodes with:

.. code-block:: bash

  roslaunch asclinic_pkg i2c_internal.launch
  roslaunch asclinic_pkg i2c_external.launch


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



Important notes on I2C usages in ROS nodes
******************************************

.. important::

  If multiple I2C devices are connected to the same I2C bus, then you **MUST** use one ROS node to interface with all those devices.

  It does **NOT work** if you create a separate ROS node for each I2C device on the same I2C bus. In such a case, the first ROS node to launch opens the I2C bus and blocks any other nodes from accessing that same I2C bus.



.. important::

  If your code attempt to communicate with an I2C device that is not physically connected to that bus, then the behaviour is not guaranteed and can be unexpected.

  The I2C drivers are written with the intention to degrade gracefully in the case of a missing device:

  * The absence of the device is detected when attempting to initialise the device and a warning is display.
  * Any further request to communicate with the device is not executed.
  * A subsequent attempt to initialise the device will again check if for the presence or absence of the device.

  Despite this intention for graceful degradation, it can still happen that the a node stalls due to a missing I2C device. This stalling can mean that the function called is continually trying to communicate with the missing, and the result can be that all I2C and GPIO buses and channels exhibit unexpected behaviour because the low level interface controllers are overloaded with the requests of the stalled node.



Step-by-step guides for the I2C device drivers
**********************************************


.. toctree::
  :maxdepth: 2
  
  workflow_i2c_pololu_smc_g2
  workflow_i2c_servo_driver_pca9685
  workflow_i2c_tof_driver_vl53l1x

