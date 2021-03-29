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
