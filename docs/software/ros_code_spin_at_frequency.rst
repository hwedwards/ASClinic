.. _ros-code-spin-at-frequency:

Code for spinning at a specified frequency (C++ and Python)
=============================================================

Requirements for this page:

* ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
* A catkin workspace and ROS package initialised as per :ref:`ros-create-catkin-ws-and-package`
* At a minimum, the simple nodes as per :ref:`ros-code-node-simple`
* Familiarity with how to :ref:`ros-run-and-launch`

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


TL;DR
*****

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

Contents for a C++ node spinning at a fixed frequency:

.. code-block:: cpp

  #include "ros/ros.h"
  #include <ros/package.h>

  // Declare "member" variables
  ros::Timer m_timer_for_counting;

  // Declare the function prototypes
  void timerCallback(const ros::TimerEvent&);

  // Implement the timer callback function
  void timerCallback(const ros::TimerEvent&)
  {
      static uint counter = 0;
      counter++;
      // Display the current counter value to the console
      ROS_INFO_STREAM("[SPINNER CPP NODE] counter = " << counter);
  }

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "spinner_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("SPINNER CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a timer
      float timer_delta_t_in_seconds = 0.5;
      m_timer_for_counting = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, false);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


Contents for a Python node spinning at a fixed frequency:

.. code-block:: python

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  class SpinnerPyNode:

    def __init__(self):
        # Initialise a counter
        self.counter = 0
        # Initialise a timer
        timer_delta_t_in_seconds = 0.5;
        rospy.Timer(rospy.Duration(timer_delta_t_in_seconds), self.timerCallback)

    # Respond to timer callback
    def timerCallback(self, event):
        self.counter += 1
        # Display the current counter value to the console
        rospy.loginfo("[SPINNER PY NODE] counter = " + str(self.counter))

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("spinner_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("SPINNER PY NODE] namespace of node = " + rospy.get_namespace());
      # Start an instance of the class
      spinner_py_node = SpinnerPyNode()
      # Spin as a single-threaded node
      rospy.spin()


Three lines for the :code:`CMakeLists.txt`, follow the pattern described in :ref:`ros-code-node-simple-add-to-cmake`:

.. code-block:: bash

  add_executable(spinner_cpp_node src/spinner_cpp_node.cpp)
  add_dependencies(spinner_cpp_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(spinner_cpp_node ${catkin_LIBRARIES})

A launch file for launching both C++ and Python spinners at the same time.

.. code-block:: html

  <launch>
      <!-- START A GROUP WITH A NAMESPACE -->
      <group ns="mrp">
          <!-- LAUNCH A "Spinner C++" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "spinner_cpp_node"
              output = "screen"
              type   = "spinner_cpp_node"
          />
          <!-- LAUNCH A "Spinner Python" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "spinner_py_node"
              output = "screen"
              type   = "spinner_py_node.py"
          />
      </group>
  </launch>


High-level description
**********************

A ROS node spinning a a specified frequency can be achieved by:

  * Initialising a ROS timer variable in the main function of the node.
  * A time delta is specified when initialising the ROS timer, and this registers that the node should repeatedly trigger the specified callback function at that time delta interval.
  * The main function then goes into a ROS spin, which trigger the timer callbacks.

.. note::

  The time delta specified for the timer is respected regardless of how long the callback function takes to complete one execution. In other words, if the callback function takes too long, then the queue of timer callbacks build up and the node's behaviour becomes be hard to predict.

.. note::

  As stated on the `ROS overview timers page <https://wiki.ros.org/roscpp/Overview/Timers>`_: "*Timers are* **not** *a realtime thread/kernel replacement, rather they are useful for things that do not have hard realtime requirements.*"


Python only: add a class definition
***********************************

Starting from the "plain_py_node.py" from :ref:`ros-code-node-simple`, add a class definition bespoke to this node, and initialise an instance of the class from the main function:

.. code-block:: python
  :emphasize-lines: 6-10,15-16

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  class SpinnerPyNode:

    def __init__(self):
        # Initialise a counter
        self.counter = 0

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("spinner_py_node")
      # Start an instance of the class
      spinner_py_node = SpinnerPyNode()
      # Spin as a single-threaded node
      rospy.spin()

The :code:`self.counter` variable is used later on for counting up the number of timer callback.


Add a callback function for when the timer triggers
***************************************************

For both C++ and Python, the callback implementation simply increments a counter and print the current value of the counter to the console.

**For C++:** add the function prototype towards the top of the file, and the timer callback function wherever you please:

.. code-block:: cpp
  :emphasize-lines: 4-5,7-14 

  #include "ros/ros.h"
  #include <ros/package.h>

  // Declare the function prototypes
  void timerCallback(const ros::TimerEvent&);

  // Implement the timer callback function
  void timerCallback(const ros::TimerEvent&)
  {
      static uint counter = 0;
      counter++;
      // Display the current counter value to the console
      ROS_INFO_STREAM("[SPINNER CPP NODE] counter = " << counter);
  }

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "spinner_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("SPINNER CPP NODE] namespace of nh = " << nh.getNamespace());
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }

.. note::

  The typical structure for C++ classes (and you can think of each ROS node you write as a separate class), is to put the includes, member variable definitions, and function prototypes into a header. This is not done header for the convenience of display one file instead of two. But as your node grows, you should consider shifting such declarations to a header file, i.e., to a file named :code:`spinner_cpp_node.h`

.. note::

  The function implementation can be placed above or below the main function, it is a matter of style. And whatever style you choose, remember that consistency of style (i.e., convention) adds tangible value through usability and maintainability.


**For Python:** add the timer callback function within the class:

.. code-block:: python
  :emphasize-lines: 12-16

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  class SpinnerPyNode:

    def __init__(self):
        # Initialise a counter
        self.counter = 0

    # Respond to timer callback
    def timerCallback(self, event):
        self.counter += 1
        # Display the current counter value to the console
        rospy.loginfo("[SPINNER PY NODE] counter = " + str(self.counter))

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("spinner_py_node")
      # Start an instance of the class
      spinner_py_node = SpinnerPyNode()
      # Spin as a single-threaded node
      rospy.spin()



Initialise a timer and connect it to the callback
*************************************************

**For C++:** the timer is initialised in the main function:

.. code-block:: cpp
  :emphasize-lines: 27-30

  #include "ros/ros.h"
  #include <ros/package.h>

  // Declare "member" variables
  ros::Timer m_timer_for_counting;

  // Declare the function prototypes
  void timerCallback(const ros::TimerEvent&);

  // Implement the timer callback function
  void timerCallback(const ros::TimerEvent&)
  {
      static uint counter = 0;
      counter++;
      // Display the current counter value to the console
      ROS_INFO_STREAM("[SPINNER CPP NODE] counter = " << counter);
  }

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "spinner_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("SPINNER CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a timer
      float timer_delta_t_in_seconds = 0.5;
      bool timer_is_one_shot = false;
      m_timer_for_counting = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, timer_is_one_shot);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


**For Python:** the timer is initialised in the :code:`__init__` of the class, which is call from the main function:

.. code-block:: python
  :emphasize-lines: 11-13

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  class SpinnerPyNode:

    def __init__(self):
        # Initialise a counter
        self.counter = 0
        # Initialise a timer
        timer_delta_t_in_seconds = 0.5;
        rospy.Timer(rospy.Duration(timer_delta_t_in_seconds), self.timerCallback, oneshot=False)

    # Respond to timer callback
    def timerCallback(self, event):
        self.counter += 1
        # Display the current counter value to the console
        rospy.loginfo("[SPINNER PY NODE] counter = " + str(self.counter))

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("spinner_py_node")
      # Start an instance of the class
      spinner_py_node = SpinnerPyNode()
      # Spin as a single-threaded node
      rospy.spin()



Add the C++ node to the C Make List and compile
***********************************************

Follow the exact same pattern described in :ref:`ros-code-node-simple-add-to-cmake`:

0. Open the :code:`CMakeLists.txt` file for editing:

  .. code-block:: bash

    cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/
    nano CMakeLists.txt

1. Add the :code:`add_executable` directive:

   .. code-block:: bash

     add_executable(spinner_cpp_node src/spinner_cpp_node.cpp)

2. Add the :code:`add_dependencies` directive:

   .. code-block:: bash

     add_dependencies(spinner_cpp_node ${catkin_EXPORTED_TARGETS})

3. Add the :code:`target_link_libraries` directive:

   .. code-block:: bash

     target_link_libraries(spinner_cpp_node ${catkin_LIBRARIES})

4. Compile

   .. code-block:: bash

     cd ~/my-robotics-system/catkin_ws/
     catkin_make


Make the Python file executable
*******************************

Add (:code:`+`) executable (:code:`x`) permissions to the spinner Python file: 

.. code-block:: bash

  chmod +x ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src/spinner_py_node.py


Launch and test
***************

Make a launch file for launching both nodes at the same time and test, following the pattern described in :ref:`ros-run-and-launch`.

The messages displayed in the console may look something like the following:

.. code-block:: console

  process[mrp/spinner_cpp_node-1]: started with pid [20000]
  process[mrp/spinner_py_node-2]: started with pid [20001]
  [ INFO] [1650283861.482913600]: SPINNER CPP NODE] namespace of nh = /mrp/spinner_cpp_node
  [ INFO] [1650283861.989005528]: [SPINNER CPP NODE] counter = 1
  [ INFO] [1650283862.487859902]: [SPINNER CPP NODE] counter = 2
  [INFO] [1650283862.694406]: [SPINNER PY NODE] counter = 1
  [ INFO] [1650283862.987786484]: [SPINNER CPP NODE] counter = 3
  [INFO] [1650283863.193749]: [SPINNER PY NODE] counter = 2
  [ INFO] [1650283863.487807749]: [SPINNER CPP NODE] counter = 4


Alternative: Fixed frequency spinning in the main
*************************************************

It is possible to spin at a fixed frequency directly within the main function of the node by using ROS loop rate type variable:

  * **Pro:** this remove the code for a timer, hence if the node performs a small task, spinning in the main can lead to a succinct, clean, and easily maintained code.
  * **Con:** this put all the functionality of the node into the main, hence if the node perform larger or multiple tasks, spinning in the main can lead to intertwined functionality that is hard to read and hard to maintain.


**For C++:** spinning in the main uses a :code:`ros::Rate` type variable, described as a "*convenience class which makes a best effort at maintaining a particular rate for a loop*", as follows:

.. code-block:: cpp
  :emphasize-lines: 13-28

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "spinner_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("SPINNER CPP NODE] namespace of nh = " << nh.getNamespace());

      // Initialise the ROS rate variable
      float loop_frequency_in_hz = 2.0;
      ros::Rate loop_rate(loop_frequency_in_hz);
      // Intialise a counter
      uint counter = 0;
      // Enter a while loop that spins while ROS is ok
      while (ros::ok)
      {
          counter++;
          // Display the current counter value to the console
          ROS_INFO_STREAM("[SPINNER CPP NODE] counter = " << counter);
          // Spin once to service anything that need servicing
          ros::spinOnce();
          // Sleep at the loop rate
          loop_rate.sleep();
      }

      // Main has ended, return 0
      return 0;
  }


**For Python:** spinning in the main uses a :code:`rospy::Rate` type variable, described as a "*convenience class which makes a best effort at maintaining a particular rate for a loop*", as follows:

.. code-block:: python
  :emphasize-lines: 10-20

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("spinner_py_node")

      # Initialise the ROS rate variable
      loop_frequency_in_hz = 2.0;
      loop_rate = rospy.Rate(loop_frequency_in_hz);
      # Intialise a counter
      counter = 0;
      # Enter a while loop that spins while ROS is ok
      while not rospy.is_shutdown():
        counter += 1
        # Display the current counter value to the console
        rospy.loginfo("[SPINNER PY NODE] counter = " + str(counter))
        loop_rate.sleep()

.. note::

  Even though :code:`rospy` does not have a "spin once" equivalent, this Python sample would still service anything that needs servicing, e.g., timer and subscriber callbacks.



References
**********

The steps detailed on this page are mostly taken from:

  * `ROS overview: roscpp logging <https://wiki.ros.org/roscpp/Overview/Logging>`_
  * `ROS overview: rospy logging <https://wiki.ros.org/rospy/Overview/Logging>`_
  * `ROS overview: roscpp timer <https://wiki.ros.org/roscpp/Overview/Timers>`_
  * `ROS overview: rospy timer <https://wiki.ros.org/rospy/Overview/Time#Timer>`_
  * `ROS overview: roscpp sleeping and rates <https://wiki.ros.org/roscpp/Overview/Time#Sleeping_and_Rates>`_
  * `ROS overview: rospy sleeping and rates <https://wiki.ros.org/rospy/Overview/Time#Sleeping_and_Rates>`_



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
