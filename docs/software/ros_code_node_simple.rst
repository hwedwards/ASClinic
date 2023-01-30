.. _ros-code-node-simple:

Code up a ROS node from scratch (C++ and Python)
================================================

Requirements for this page:

  * ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
  * A catkin workspace and ROS package initialised as per :ref:`ros-create-catkin-ws-and-package`

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2


TL;DR
*****

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

.. code-block:: bash

  # CHANGE TO THE SOURCE DIRECTORY FOR WRITING ROS NODES WITHIN YOUR ROS PACKAGE
  cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src
  
  # CREATE AND EDIT A C++ FILE WITH THE CONTENTS FROM THIS PAGE
  nano plain_cpp_node.cpp

  # CREATE AND EDIT A C++ FILE WITH THE CONTENTS FROM THIS PAGE
  nano plain_py_node.py


Contents for a plain C++ node:

.. code-block:: cpp

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("PLAIN CPP NODE] namespace of nh = " << nh.getNamespace());
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


Contents for a plain Python node:

.. code-block:: python

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("plain_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("PLAIN PY NODE] namespace of node = " + rospy.get_namespace());
      # Spin as a single-threaded node
      rospy.spin()





Python only: interpreter and encoding directives
************************************************

As with any Python file, you can use the first lines of the script to specify the interpreter to be used, for example:

.. code-block:: python

  #!/usr/bin/env python

Followed by the encoding, for example:

.. code-block:: python

  # -*- coding: utf-8 -*-

.. note::

  If using ROS melodic or earlier, and you want to use a python 3.x interpreter, then you need to use the following interpreter directive:

  .. code-block:: python

    #!/usr/bin/env python3

  (Unless you have done some extra configuration to make Python 3 the default on your system.)


Import the appropriate ROS API (Application Programming Interface)
******************************************************************

The ROS API must be included / imported so that you can access the ROS libraries and functionality for that language (C++ or Python).

**For C++:** the API includes are:

.. code-block:: cpp

  #include "ros/ros.h"
  #include <ros/package.h>

**For Python:** the API import is:

.. code-block:: python
  :emphasize-lines: 4

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy


Add the main function
*********************

**For C++:** add the typical main function:

.. code-block:: cpp
  :emphasize-lines: 4-8

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Main has ended, return 0
      return 0;
  }

**For Python:** add the typical main function:

.. code-block:: python
  :emphasize-lines: 6

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  if __name__ == '__main__':


Initialise the node
*******************

For both C++ and Python, the API call for initialising the node requires an argument that specifies the name of the node, and this name is how the node is identified by ROS.

.. important::

  You can specify any name for a node, but as with any naming options, choose something meaningful and easy to identify. A good convention is to make the node name the same as the file name.

**For C++:** add the :code:`ros::init` call to initialise the node:

.. code-block:: cpp
  :emphasize-lines: 6-7

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Main has ended, return 0
      return 0;
  }


**For Python:** add the :code:`rospy.init_node` call to initialise the node:

.. code-block:: python
  :emphasize-lines: 7-8

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("plain_py_node")


C++ only: start the node
************************

In order to actually start a C++ node, the :code:`ros::start()` function needs to be called. However, it is common practice to instead create a ROS node handle as follows because this will call :code:`ros::start()` behind-the-scenes:

.. code-block:: cpp
  :emphasize-lines: 8-11

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Main has ended, return 0
      return 0;
  }

What happens behind-the-scenes is that the first node handle created calls :code:`ros::start()` for this node. And destroying the last remaining node handle will call :code:`ros::shutdown()`. Hence you could directly start and showdown as follows:

.. code-block:: cpp
  :emphasize-lines: 8-11

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Start the node
      ros::start();
      // Shutdown the node
      ros::start();
      // Main has ended, return 0
      return 0;
  }

.. note::

  You can call :code:`ros::shutdown()` from anywhere within your node, and it will cancel all the publishers, subscribers, services, parameters of that node.

.. note::

  The line :code:`ros::NodeHandle nh("~");` creates the variable :code:`nh` to be of type :code:`ros::NodeHandle` which point to the this node.

  You can think of the node handle as simple a string comprised of the namespace of the node and the name of the node, i.e., of the form :code:`/<namespace_of_node>/plain_cpp_node`.


Display the namespace of the node
*********************************

The namespace of a node is a very important attribute for you to create an ecosystem of ROS nodes for performing complex robotics tasks (see the wordy description of :ref:`ros-key-elements-namespaces` for more details). Incorrect or unexpected namespaces can also be the source of "bugs" and undesirable behaviour.

We recommend to always display to the console the namespace of a node immediately after the node is started.

**For C++:** use :code:`ROS_INFO_STREAM(...)` to display the output of :code:`getNamespace()`:

.. code-block:: cpp
  :emphasize-lines: 10-11

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("PLAIN CPP NODE] namespace of nh = " << nh.getNamespace());
      // Main has ended, return 0
      return 0;
  }

**For Python:** add the :code:`rospy.loginfo(...)` call to display the output of :code:`rospy.get_namespace()`:

.. code-block:: python
  :emphasize-lines: 9-10

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("plain_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("PLAIN PY NODE] namespace of node = " + rospy.get_namespace());



Keep the alive by spinning in a while loop
******************************************

The code so far creates the node, then immediately kills the node and exits. Obviously we want our nodes to stay alive as long as our robotic system is operating. Keep the node alive is referred to in ROS as spinning.

**For C++:** add a while loop that continues while ROS is ok and spins once for every execution of the while loop.

.. code-block:: cpp
  :emphasize-lines: 12-16

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("PLAIN CPP NODE] namespace of nh = " << nh.getNamespace());
      // Enter a while loop that spins while ROS is ok
      while (ros::ok)
      {
          ros::spinOnce();
      }
      // Main has ended, return 0
      return 0;
  }

.. important::

  **For Python:** there is not a direct equivalent of :code:`ros::spinOnce()`, but reading this section is important for understanding the next section. The equivalent of :code:`while (ros::ok)` is :code:`while not rospy.is_shutdown():`


The :code:`spinOnce()` function is essentially a directive for ROS to check for and execute anything related to this node, including:

  * Publishing any messages queued for publication.
  * Responding to any message waiting in subscriber queues.
  * Responding to any incoming service requests.
  * Executing any timer callbacks.


Keep the alive with the ROS spin function
*****************************************

Both the C++ and Python API provide a convenience function for the "while ok, spin once" described in the previous section.

**For C++:** add the :code:`ros::spin()` call to keep the node alive:

.. code-block:: cpp
  :emphasize-lines: 12-13

  #include "ros/ros.h"
  #include <ros/package.h>

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "plain_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("PLAIN CPP NODE] namespace of nh = " << nh.getNamespace());
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


**For Python:** add the :code:`rospy.spin()` call to keep the node alive:

.. code-block:: python
  :emphasize-lines: 11-12

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("plain_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("PLAIN PY NODE] namespace of node = " + rospy.get_namespace());
      # Spin as a single-threaded node
      rospy.spin()

The ROS spin commands are essentially the same as the "while ok, spin once" code explained in the previous section. Hence the ROS spin commands:

  * Block the execution of the main function while ROS is ok.
  * Monitor for and trigger execution of any callbacks for this node when necessary.





References
**********

The steps detailed on this page are mostly taken from:

  * `ROS overview: Initialization and shutdown <https://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown>`_
  * `ROS overview: callbacks and spinning <https://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning>`_
  * `ROS Cpp Class Reference for ros::NodeHandle <https://docs.ros.org/en/noetic/api/roscpp/html/classros_1_1NodeHandle.html>`_
