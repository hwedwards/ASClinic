.. _workflow-gpio-single-pin:

ROS Interface with a Single GPIO Pin
====================================

Once the device is physically connected to a GPIO, and before attempting to interface using ROS, you should use the command line tool to check the device is properly connected.
See the :ref:`GPIO page <comm-protocol-GPIO>` for details of the command line tools, and see the :ref:`SBC page <single-board-computers>` for details of the GPIO pins for the relevant SBC.


For example, if the input is connected to pin 7 of the J21 expansion header of the Jetson Xavier NX, then this corresponds to line 148 of the :code:`gpiochip0` chip (:ref:`see the table here for this pin-to-line-number mapping <sbc-jetson-xavier-nx-pin-mapping>`).
Hence you can read the value of pin 7 with the command:

.. code-block:: bash

  sudo gpioget gpiochip0 148

To monitor a line for multiple of a particular event, use the command:

.. code-block:: bash

  sudo gpiomon --num-events=3 --rising-edge gpiochip0 148

To interface with a single GPIO pin via a ROS node, there are two templates provided:

* The file named :code:`template_gpio_event_triggered.cpp`: this template monitors the GPIO line for events (i.e., a rising and/or falling edge), and publishes a message each time an event occurs.
* The file named :code:`template_gpio_polling.cpp`: this template check the value of the GPIO line at a fixed frequency.

Both files are located in the repository at the relative file path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/nodes/

Both files are extensively commented and the comments serve as the documentation for how to edit the file to implement your use case.
The GPIO templates are C++ files because the library used for interfacing with the GPIO pins is C++ based.
There does exist Python bindings for the GPIO library, but that is not currently developed as part of this repository.

Both template nodes are launched by file :code:`template_gpio.launch`.
You should review the launch file to check how it is structure to add a parameter to each node upon launch (see also the note below).
Then you can launch the template GPIO nodes with:

.. code-block:: bash

  roslaunch asclinic_pkg template_gpio.launch



.. note::

  **GPIOD Library - C++ Documentation:**
  The following two websites both provide a comprehensive list of all the functions offered by the :code:`gpiod` library for monitoring a GPIO pin:

    * `gpiod documentation hosted by dlang <https://libgpiod-dlang.dpldocs.info/gpiod.html>`_
    * `gpiod doxygen documentation hosted by lane-fu <https://www.lane-fu.com/linuxmirror/libgpiod/doc/html/index.html>`_



.. note::
  **Adding parameters to nodes in the launch file:**
  The line number to monitor is specified using a :code:`<param/>` parameter tag in the launch file.
  Hence, to change the line number being monitored, you simply need to change the line number parameter in the launch file and re-launch the node.

  * This has the benefit that you can change the line number without needing to recompile the code.
  * This has the disadvantage that you cannot specify the line number parameter when using the :code:`rosrun` command to start the node.
  * Be careful that two separate nodes cannot access the same GPIO line because the first nodes that runs and open are particular line blocks all other process from accessing that line.




.. note::
  **Compilation flag for nodes using GPIOD:**
  When you copy either template C++ file, you will need to add it to the :code:`CMakeLists.txt` file in the repository at the relative file path:

  .. code-block:: bash

    catkin_ws/src/asclinic_pkg/CMakeLists.txt

  Simply duplicate and accordingly edit the lines where the name of the respective template GPIO C++ file appears.
  Most important is that the :code:`target_link_libraries(...)` needs the compile link flag :code:`-lgpiod` at the end.

  .. code-block:: bash

    target_link_libraries(template_gpio ${catkin_LIBRARIES} -lgpiod)
