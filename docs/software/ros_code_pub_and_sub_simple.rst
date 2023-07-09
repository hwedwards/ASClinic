.. _ros-code-pub-and-sub-simple:

Code for a simple publisher and subscriber (C++ and Python)
=============================================================

Requirements for this page:

* ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
* A catkin workspace and ROS package initialised as per :ref:`ros-create-catkin-ws-and-package`
* At a minimum, the simple nodes as per :ref:`ros-code-node-simple`
* Preferably the spinning nodes as per :ref:`ros-code-spin-at-frequency`
* Familiarity with how to :ref:`ros-run-and-launch`

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2

.. note::

  The steps for a publisher and subscriber are mixed together because many steps are the same and it is recommended that you have the code for both nodes open and write them at the same time.

  The section headings indicate whether step is relevant for a (PUBLISHER), a (SUBSCRIBER) or (BOTH)


TL;DR - (PUBLISHER)
*******************

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

Contents for **a simple C++ publisher**, highlighted lines are all those relevant for the publisher:

.. code-block:: cpp
  :emphasize-lines: 3,7,17-20,31-37

  #include "ros/ros.h"
  #include <ros/package.h>
  #include "std_msgs/UInt32.h"

  // Declare "member" variables
  ros::Timer m_timer_for_publishing;
  ros::Publisher m_publisher;

  // Declare the function prototypes
  void timerCallback(const ros::TimerEvent&);

  // Implement the timer callback function
  void timerCallback(const ros::TimerEvent&)
  {
      static uint counter = 0;
      counter += 2;
      // Build and publish a message
      std_msgs::UInt32 msg;
      msg.data = counter;
      m_publisher.publish(msg);
  }

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "publisher_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("[PUBLISHER  CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a node handle to the group namespace
      std::string ns_for_group = ros::this_node::getNamespace();
      ros::NodeHandle nh_for_group(ns_for_group);
      // Initialise a publisher relative to the group namespace
      uint32_t queue_size = 10;
      bool should_latch = false;
      m_publisher = nh_for_group.advertise<std_msgs::UInt32>("great_topic", queue_size, should_latch);
      // Initialise a timer
      float timer_delta_t_in_seconds = 0.5;
      m_timer_for_publishing = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, false);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


Contents for **a simple Python publisher**, highlighted lines are all those relevant for the publisher:

.. code-block:: python
  :emphasize-lines: 5,7,10-11,21-22,29-30

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy
  from std_msgs.msg import UInt32

  class PublisherPyNode:

    def __init__(self):
        # Initialise a publisher
        self.m_publisher = rospy.Publisher(this_nodes_namespace + "/great_topic", UInt32, queue_size=10, latch=False)
        # Initialise a counter
        self.counter = 1
        # Initialise a timer
        timer_delta_t_in_seconds = 0.5;
        rospy.Timer(rospy.Duration(timer_delta_t_in_seconds), self.timerCallback)

    # Implement the timer callback
    def timerCallback(self, event):
        self.counter += 2
        # Publish a message
        self.m_publisher.publish(self.counter)

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("publisher_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("PUBLISHER  PY  NODE] namespace of node = " + rospy.get_namespace())
      # Put the namespace into a global variable for this script
      global this_nodes_namespace = rospy.get_namespace()
      # Start an instance of the class
      publisher_py_node = PublisherPyNode()
      # Spin as a single-threaded node
      rospy.spin()


Three lines for the :code:`CMakeLists.txt`, follow the pattern described in :ref:`ros-code-node-simple-add-to-cmake`:

.. code-block:: bash

  add_executable(publisher_cpp_node src/publisher_cpp_node.cpp)
  add_dependencies(publisher_cpp_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(publisher_cpp_node ${catkin_LIBRARIES})


.. note::

  Publisher nodes do **NOT** need to be a constant frequency spinner. We use such a spinner as a basis for this tutorial purely as a method to continually publish messages.



TL;DR - (SUBSCRIBER)
********************

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

Contents for **a simple C++ subscriber**, highlighted lines are all those relevant for the subscriber:

.. code-block:: cpp
  :emphasize-lines: 3,5-6,8-15,25-30

  #include "ros/ros.h"
  #include <ros/package.h>
  #include "std_msgs/UInt32.h"

  // Declare the function prototypes
  void subscriberCallback(const std_msgs::UInt32& msg);

  // Implement the subscriber callback function
  void subscriberCallback(const std_msgs::UInt32& msg)
  {
      // Extract the data from the message
      uint32_t this_data = msg.data;
      // Display the data
      ROS_INFO_STREAM("[SUBSCRIBER CPP NODE] received message with data = " << this_data);
  }

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "subscriber_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("[SUBSCRIBER CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a node handle to the group namespace
      std::string ns_for_group = ros::this_node::getNamespace();
      ros::NodeHandle nh_for_group(ns_for_group);
      // Initialise a subscriber relative to the group namespace
      uint32_t queue_size = 1;
      ros::Subscriber local_subscriber = nh_for_group.subscribe("great_topic", queue_size, subscriberCallback);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


Contents for **a simple Python subscriber**, highlighted lines are all those relevant for the subscriber:

.. code-block:: python
  :emphasize-lines: 5,7,10-11,13-18,25-28

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy
  from std_msgs.msg import UInt32

  class SubscriberPyNode:

    def __init__(self):
        # Initialise a subscriber
        rospy.Subscriber(this_nodes_namespace + "/great_topic", UInt32, self.subscriberCallback, queue_size=1)

    # Implement the subscriber callback
    def subscriberCallback(self, msg):
        # Extract the data from the message
        this_data = msg.data
        # Display the data
        rospy.loginfo("[SUBSCRIBER PY  NODE] received message with data = " + str(this_data)

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("plain_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("[SUBSCRIBER PY  NODE] namespace of node = " + rospy.get_namespace())
      # Put the namespace into a global variable for this script
      global this_nodes_namespace = rospy.get_namespace()
      # Start an instance of the class
      publisher_py_node = SubscriberPyNode()
      # Spin as a single-threaded node
      rospy.spin()


Three lines for the :code:`CMakeLists.txt`, follow the pattern described in :ref:`ros-code-node-simple-add-to-cmake`:

.. code-block:: bash

  add_executable(subscriber_cpp_node src/publisher_cpp_node.cpp)
  add_dependencies(subscriber_cpp_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(subscriber_cpp_node ${catkin_LIBRARIES})



.. _ros-code-pub-and-sub-simple-tldr-launch:

TL;DR - launch (BOTH)
*********************

A launch file for launching both C++ and Python publishers and subscribers at the same time. In Section ABC below we discuss what output to expect.

.. code-block:: html

  <launch>
      <!-- START A GROUP WITH A NAMESPACE -->
      <group ns="mrp">
          <!-- LAUNCH A "Publisher C++" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "publisher_cpp_node"
              output = "screen"
              type   = "publisher_cpp_node"
          />
          <!-- LAUNCH A "Publisher Python" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "publisher_py_node"
              output = "screen"
              type   = "publisher_py_node.py"
          />
          <!-- LAUNCH A "Subscriber C++" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "subscriber_cpp_node"
              output = "screen"
              type   = "subscriber_cpp_node"
          />
          <!-- LAUNCH A "Subscriber Python" NODE -->
          <node
              pkg    = "my_robotics_pkg"
              name   = "subscriber_py_node"
              output = "screen"
              type   = "subscriber_py_node.py"
          />
      </group>
  </launch>





High-level description (BOTH)
*****************************

**Starting files:** in order to follow this tutorial, it is recommended that you use the following four files as a starting point:

* One file for each of the C++ and Python publisher, using the :ref:`ros-code-spin-at-frequency` as the basis, respectively named:

  * :code:`publisher_cpp_node.cpp`
  * :code:`publisher_py_node.py`

* One file for each of the C++ and Python subscriber, using the :ref:`ros-code-node-simple` as the basis, respectively named:

  * :code:`subscriber_cpp_node.cpp`
  * :code:`subscriber_py_node.py`



**Publishing** is managed in ROS through **Publisher** type variables. Initialising a publisher involves specifying as a minimum:

  * The name of the topic on which the messages are to be published.
  * The type of message to be published.
  
You then use this publisher variable to publish messages from any of the function within your node.

**Subscribing** is managed in ROS through **Subscriber** type variables. Initialising a subscriber involves specifying as a minimum:

  * The name of the topic on which to listen for messages.
  * The callback function to use when any message is received on the topic.
  * The type of message expected.

The subscriber variable generally does not need to be access again after it is initialised. As long as the subscriber variable still exists, the subscriber callback responds to all message on the specified topic. The existence of a subscriber variable is generally tied to the spinning of the main function.



.. important::

  Publishers and subscribers on the same topic **MUST** be specified with the **SAME** message type. A difference in message type causes error messages at run time and unexpected behaviour of your robot.



Import the header for the message type (BOTH)
*********************************************

For both C++ and Python, you need to import the header for every message type used in your node. Headers for the primitive message types are defined in the so-called :code:`std_msgs` (i.e., standard messages). We use an 32-bit unsigned integer for this tutorial.

**For C++:** import the :code:`UInt32` header:

.. code-block:: cpp
  
  #include "std_msgs/UInt32.h"


**For Python:** import the :code:`UInt32` header:

.. code-block:: python

  from std_msgs.msg import UInt32


List of the standard message (:code:`std_msgs`) types (BOTH)
************************************************************

A full list of the message types defined in :code:`std_msgs` can be found in on `the ROS Wiki page for std_msgs <https://wiki.ros.org/std_msgs>`_, and the following table lists some of the most useful data types.

.. list-table::
  :widths: auto
  :width: 100
  :header-rows: 1
  :stub-columns: 0
  :align: center

  * - **std_msgs**
    - **Description**

  * - :code:`Bool`
    - Boolean

  * - :code:`Char`
    - Character

  * - :code:`Empty`
    - Literally nothing

  * - :code:`Float32`

      :code:`Float64`
    - Floating point number of size 32 or 64 bits

  * - :code:`Int8`

      :code:`Int16`

      :code:`Int32`

      :code:`Int64`
    - Integer of size 8, 16, 32, or 64 bits

  * - :code:`String`
    - A sequence of characters

  * - :code:`UInt8`

      :code:`UInt16`

      :code:`UInt32`

      :code:`UInt64`
    - Unsigned integer of size 8, 16, 32, or 64 bits

.. note::

  This tutorial used a :code:`UInt32` message type. To use a different message type, simply replace all occurrences of :code:`UInt32` with the desired message type from the **std_msgs** column of the table.

.. important::

  As noted on `std_msgs wiki page <https://wiki.ros.org/std_msgs>`_, these standard message types are **NOT intended for "long-term" usage** because they lack semantic information.

  The ABC page provides details for how you can define custom message types for adding semantic information to the ROS message that you build and send.



C++ only: initialise a node handle for the node's namespace (BOTH)
**********************************************************************

As both the publisher and subscriber are initialised relative to a :code:`NodeHandle` type object, we need to initialise such an object that points to the desired namespace of the publishing the subscribing topic.

.. code-block:: cpp
  :emphasize-lines: 9-11

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "publisher_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("[PUBLISHER  CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a node handle to the group namespace
      std::string ns_for_group = ros::this_node::getNamespace();
      ros::NodeHandle nh_for_group(ns_for_group);
      // Initialise a timer
      float timer_delta_t_in_seconds = 0.5;
      m_timer_for_publishing = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, false);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }

.. note::

  In this tutorial we are using the node's namespace for the topic being published/subscribed. Section ABC described alternative namespaces you can consider.



Python only: make the namespace a global variable (BOTH)
***********************************************************

As both the publisher and subscriber are to be initialised in the node class, we choose to name the node's namespace a global variable from where the node it initialised in the :code:`__main__` function of the Python script.

.. code-block:: python
  :emphasize-lines: 6-7

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("publisher_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("PUBLISHER  PY  NODE] namespace of node = " + rospy.get_namespace());
      # Put the namespace into a global variable for this script
      global this_nodes_namespace = rospy.get_namespace()

.. note::

  In this tutorial we are using the node's namespace for the topic being published/subscribed. Section ABC described alternative namespaces you can consider.



C++ only: declare the publisher as a member variable (PULISHER)
***************************************************************

As the :code:`ros::Publisher` type varaible is accessed by multiple functions, it needs to be declared as a member variable of the node.

.. code-block:: cpp
  :emphasize-lines: 7

  #include "ros/ros.h"
  #include <ros/package.h>
  #include "std_msgs/UInt32.h"

  // Declare "member" variables
  ros::Timer m_timer_for_publishing;
  ros::Publisher m_publisher;



Initialise the publisher variable (PULISHER)
********************************************

Initialising a publisher requires you to specify the following:

* The name of the topic.
* The namespace of the topic:

  * For C++ this is taken from the :code:`NodeHandle` type object used.
  * For python this is specified as part of the topic name.

* The message type of the topic.
* The size of the publisher queue. This specifies how many messages are buffered in the publisher's outgoing queue. For example, this prevents messages being lost during a period where your nodes is calling the "publish" function faster than the transport layer can actually encapsulate and send those messages. When the queue is exceeded, the oldest messaged are dropped first.
* The "latch" option. When the "latch" option is "true", i.e., latching is enabled, then then the last message on that topic is saved, and will be sent to any future subscriber to that topic.

**For C++:** Intialise the publisher within the main function:

.. code-block:: cpp
  :emphasize-lines: 12-15

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "publisher_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("[PUBLISHER  CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a node handle to the group namespace
      std::string ns_for_group = ros::this_node::getNamespace();
      ros::NodeHandle nh_for_group(ns_for_group);
      // Initialise a publisher relative to the group namespace
      uint32_t queue_size = 10;
      bool should_latch = false;
      m_publisher = nh_for_group.advertise<std_msgs::UInt32>("great_topic", queue_size, should_latch);
      // Initialise a timer
      float timer_delta_t_in_seconds = 0.5;
      m_timer_for_publishing = nh.createTimer(ros::Duration(timer_delta_t_in_seconds), timerCallback, false);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }


**For Python:** Intialise the publisher within the :code:`__init__` function of the class:

.. code-block:: python
  :emphasize-lines: 7,9-11,25-26

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy
  from std_msgs.msg import UInt32

  class PublisherPyNode:

    def __init__(self):
        # Initialise a publisher
        self.m_publisher = rospy.Publisher(this_nodes_namespace + "/great_topic", UInt32, queue_size=10, latch=False)
        # Initialise a counter
        self.counter = 1
        # Initialise a timer
        timer_delta_t_in_seconds = 0.5;
        rospy.Timer(rospy.Duration(timer_delta_t_in_seconds), self.timerCallback)

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("publisher_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("PUBLISHER  PY  NODE] namespace of node = " + rospy.get_namespace());
      # Put the namespace into a global variable for this script
      global this_nodes_namespace = rospy.get_namespace()
      # Start an instance of the class
      publisher_py_node = PublisherPyNode()
      # Spin as a single-threaded node
      rospy.spin()



Publish a message (from the timer callback function) (PULISHER)
***************************************************************

The publisher variable can now be used to publish message from within any function of your node. For the purpose of this tutorial, with use a constant frequency timer to continually publish messages with an increasing count. In order to publish a message, you first need to construct a local variable that is the correct type of message for the publisher variable.

**For C++:** All of the standard message types store the message data in a :code:`data` field:

.. code-block:: cpp
  :emphasize-lines: 6-9

  // Implement the timer callback function
  void timerCallback(const ros::TimerEvent&)
  {
      static uint counter = 0;
      counter += 2;
      // Build and publish a message
      std_msgs::UInt32 msg;
      msg.data = counter;
      m_publisher.publish(msg);
  }

**For Python:** The standard message types can be published without needing to explicitly construct the message. The data (in this case the value of :code:`self.counter`) is put into a :code:`data` field as part of the publish function:

.. code-block:: python
  :emphasize-lines: 4-5

  # Implement the timer callback
  def timerCallback(self, event):
      self.counter += 2
      // Publish a message
      self.m_publisher.publish(self.counter)


.. note::

  The code for implementing a timer to trigger this callback is not included for convenience. See :ref:`ros-code-spin-at-frequency` for all details.



Initialise the subscriber variable (SUBSCRIBER)
***********************************************

Initialising a subscriber requires you to specify the following:

* The name of the topic.
* The namespace of the topic:

  * For C++ this is taken from the :code:`NodeHandle` type object used.
  * For python this is specified as part of the topic name.

* The message type of the topic.
* The size of the subscriber queue. This specifies how many messages are buffered in the subscriber's incoming queue. For example, this prevents messages being lost during a period where messages are arriving faster than your nodes can process those messages in your subscriber callback function. When the queue is exceeded, the oldest messaged are dropped first.

  * A queue size of :code:`0` is interpreted as an infinite queue, which is dangerous, do **NOT** do this.

  * A queue size of :code:`1` means that the subscriber call back function is always being provided with the most recent message.



**For C++:** Intialise the subscriber within the main function:

.. code-block:: cpp
  :emphasize-lines: 12-14

  int main(int argc, char* argv[])
  {
      // Initialise the node
      ros::init(argc, argv, "subscriber_cpp_node");
      // Start the node by initialising a node handle
      ros::NodeHandle nh("~");
      // Display the namespace of the node handle
      ROS_INFO_STREAM("[SUBSCRIBER CPP NODE] namespace of nh = " << nh.getNamespace());
      // Initialise a node handle to the group namespace
      std::string ns_for_group = ros::this_node::getNamespace();
      ros::NodeHandle nh_for_group(ns_for_group);
      // Initialise a subscriber relative to the group namespace
      uint32_t queue_size = 1;
      ros::Subscriber local_subscriber = nh_for_group.subscribe("great_topic", queue_size, subscriberCallback);
      // Spin as a single-threaded node
      ros::spin();
      // Main has ended, return 0
      return 0;
  }

.. note::

  The C++ initialisation of the subscriber does not explicitly specify the message type. This is specified in the next step by the argument of the callback function.



**For Python:** Intialise the subscriber within the :code:`__init__` function of the class:

.. code-block:: python
  :emphasize-lines: 10-11

  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy
  from std_msgs.msg import UInt32

  class SubscriberPyNode:

    def __init__(self):
        # Initialise a subscriber
        rospy.Subscriber(this_nodes_namespace + "/great_topic", UInt32, self.subscriberCallback, queue_size=1)

  if __name__ == '__main__':
      # Initialise the node
      rospy.init_node("plain_py_node")
      # Display the namespace of the node handle
      rospy.loginfo("[SUBSCRIBER PY  NODE] namespace of node = " + rospy.get_namespace())
      # Put the namespace into a global variable for this script
      global this_nodes_namespace = rospy.get_namespace()
      # Start an instance of the class
      publisher_py_node = SubscriberPyNode()
      # Spin as a single-threaded node
      rospy.spin()



Add the subscriber callback function for when messages are received (SUBSCRIBER)
********************************************************************************

For both C++ and Python, the callback implementation in this tutorial simply displays the message data received.

**For C++:** add the function prototype towards the top of the file, and the subscriber callback function wherever you please:

.. code-block:: cpp
  :emphasize-lines: 5-6,8-15

  #include "ros/ros.h"
  #include <ros/package.h>
  #include "std_msgs/UInt32.h"

  // Declare the function prototypes
  void subscriberCallback(const std_msgs::UInt32& msg);

  // Implement the subscriber callback function
  void subscriberCallback(const std_msgs::UInt32& msg)
  {
      // Extract the data from the message
      uint32_t this_data = msg.data;
      // Display the data
      ROS_INFO_STREAM("[SUBSCRIBER CPP NODE] received message with data = " << this_data);
  }

.. note::

  For C++, the argument of the callback function defines the message type expected by the subscriber, i.e., :code:`std_msgs::UInt32`. If a publisher of this topic specifies a different message type, then an error message is displayed at run time.


**For Python:** add the timer callback function within the class:

.. code-block:: python
  :emphasize-lines: 7-12

  class SubscriberPyNode:

    def __init__(self):
        # Initialise a subscriber
        rospy.Subscriber(this_nodes_namespace + "/great_topic", UInt32, self.subscriberCallback, queue_size=1)

    # Implement the subscriber callback
    def subscriberCallback(self, msg):
        # Extract the data from the message
        this_data = msg.data
        # Display the data
        rospy.loginfo("[SUBSCRIBER PY  NODE] received message with data = " + str(this_data)



Add the C++ nodes to the C Make List and compile (BOTH)
*******************************************************

Follow the exact same pattern described in :ref:`ros-code-node-simple-add-to-cmake`:

0. Open the :code:`CMakeLists.txt` file for editing:

  .. code-block:: bash

    cd ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/
    nano CMakeLists.txt

1. Add the :code:`add_executable` directive:

   .. code-block:: bash

     add_executable(publisher_cpp_node src/publisher_cpp_node.cpp)
     add_executable(subscriber_cpp_node src/subscriber_cpp_node.cpp)

2. Add the :code:`add_dependencies` directive:

   .. code-block:: bash

     add_dependencies(publisher_cpp_node ${catkin_EXPORTED_TARGETS})
     add_dependencies(subscriber_cpp_node ${catkin_EXPORTED_TARGETS})

3. Add the :code:`target_link_libraries` directive:

   .. code-block:: bash

     target_link_libraries(publisher_cpp_node ${catkin_LIBRARIES})
     target_link_libraries(subscriber_cpp_node ${catkin_LIBRARIES})

4. Compile

   .. code-block:: bash

     cd ~/my-robotics-system/catkin_ws/
     catkin_make


Make the Python file executable (BOTH)
**************************************

Add (:code:`+`) executable (:code:`x`) permissions to the Python files:

.. code-block:: bash

  chmod +x ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src/publisher_py_node.py
  chmod +x ~/my-robotics-system/catkin_ws/src/my_robotics_pkg/src/subscriber_py_node.py


Launch and test (BOTH)
**********************

Make a launch file for launching all four nodes at the same time (see :ref:`ros-code-pub-and-sub-simple-tldr-launch` for launch file contents) and test, following the pattern described in :ref:`ros-run-and-launch`.

The messages displayed in the console may look something like the following:

.. code-block:: console

  process[mrp/publisher_cpp_node-1]: started with pid [20000]
  process[mrp/subscriber_cpp_node-1]: started with pid [20001]
  process[mrp/publisher_py_node-1]: started with pid [20002]
  process[mrp/subscriber_py_node-1]: started with pid [20003]
  [ INFO] [1650283861.482913]: [PUBLISHER  CPP NODE] namespace of nh = /mrp/publishder_cpp_node
  [ INFO] [1650283861.989005]: [SUBSCRIBER CPP NODE] namespace of nh = /mrp/subscriber_cpp_node
  [ INFO] [1650283862.487859]: [PUBLISHER  PY  NODE] namespace of nh = /mrp/publishder_py_node
  [ INFO] [1650283862.694406]: [SUBSCRIBER PY  NODE] namespace of nh = /mrp/subscriber_py_node
  [ INFO] [1650283862.987786]: [SUBSCRIBER CPP NODE] received message with data = 2
  [ INFO] [1650283863.193749]: [SUBSCRIBER PY  NODE] received message with data = 2
  [ INFO] [1650283863.487807]: [SUBSCRIBER CPP NODE] received message with data = 1



Topic namespace options (BOTH)
******************************

The beauty and challenge of namespacing is that you have full freedom to specify namespaces as you please. This section describes three "natural" options for choosing the namespaces of your ROS topics. The appropriate option to use depends on how the topic fits into your overall ROS architecture and depends on what scalability requirements you have. As your ROS ecosystem grows and you become more proficient with ROS, use will inevitably face important design choices for your namespaces.



Relative to the group namespace
###############################

.. note::

  This is the option used in the descriptions of this page. It is the recommended "default" option.

The group namespace of the node is used as the namespace for the topics. Topic's with this option have a namespace and name of the form:

.. code-block:: bash

  /group_namespace/great_topic

* The main advantage of this method is that you can use groups in your launch files to keep namespaces separate for separate robots without needing to change anything about the C++/Python code.

**For C++:** this is achieved by defining a node handle that points to the namespace, and then publishing/subscribing relative to this node handle. The key snippets of code for this are:

.. code-block:: cpp

  // Initialise a node handle to the group namespace
  std::string ns_for_group = ros::this_node::getNamespace();
  ros::NodeHandle nh_for_group(ns_for_group);

.. code-block:: cpp

  // Initialise a publisher relative to the group namespace
  uint32_t queue_size = 10;
  bool should_latch = false;
  m_publisher = nh_for_group.advertise<std_msgs::UInt32>("great_topic", queue_size, should_latch);

.. code-block:: cpp

  // Initialise a subscriber relative to the group namespace
  uint32_t queue_size = 1;
  ros::Subscriber local_subscriber = nh_for_group.subscribe("great_topic", queue_size, subscriberCallback);


**For Python:** this is achieved by getting the string of namespace, and then using this string to construct the publisher/subscriber topic. The key snippets of code for this are:

.. code-block:: python

  # Put the namespace into a global variable for this script
  global this_nodes_namespace = rospy.get_namespace()


.. code-block:: python

  # Initialise a publisher
  self.m_publisher = rospy.Publisher(this_nodes_namespace + "/great_topic", UInt32, queue_size=10, latch=False)


.. code-block:: python

  # Initialise a subscriber
  rospy.Subscriber(this_nodes_namespace + "/great_topic", UInt32, self.subscriberCallback, queue_size=1)


Relative to the a hard-coded global namespace
#############################################

A hard-coded string is used as the namespace for the topics. Topic's with this option have a namespace and name of the form:

.. code-block:: bash

  /my_global_namespace/great_topic

* The main advantage of this method is that you can guarantee that multiple nodes are publishing/subscribing within the same namespace regardless of how those nodes are launched.

**For C++:** this is achieved by defining a node handle with the hard-coded string, and then publishing/subscribing relative to this node handle. The key snippets of code for this are:

.. code-block:: cpp

  // Initialise a node handle the "global" namespace using a hard-coded string
  ros::NodeHandle nh_for_global("/my_global_namespace");

Publishing and subscribing are then identical to above, except that you use :code:`nh_for_global` instead of :code:`nh_for_group`.


**For Python:** this is achieved by hard-coding a string of the global namespace, and then using this string to construct the publisher/subscriber topic. The key snippets of code for this are:

.. code-block:: python

  # Put the hard-coded namespace string into a global variable for this script
  global my_global_namespace = "/my_global_namespace"

Publishing and subscribing are then identical to above, except that you use :code:`my_global_namespace` instead of :code:`this_nodes_namespace`.

.. important::

  It is **VERY IMPORTANT** for this gobal namespace approach that the hard-coded string begins with a backslach character, i.e., with :code:`/`, i.e., as you see in hard-coded string examples: :code:`"/my_global_namespace"`



Relative to the node
####################

The node's namespace and name form the namespace for the  hard-coded string is used as the namespace for the topics. Topic's with this option have a namespace and name of the form:

.. code-block:: bash

  /group_namespace/publisher_cpp_node/great_topic
  /group_namespace/subscriber_cpp_node/great_topic
  /group_namespace/publisher_py_node/great_topic
  /group_namespace/subscriber_py_node/great_topic

* The main advantage of this method is that a topic is unlikely to be accidentally duplicated by another node.
* The main disadvantage of this method is that you need to put more thought into connecting topics across multiple nodes. As you see from the four topic namespace and names given just above this, using this option of this tutorial would produce four different :code:`great_topic` topics that do not communicate with each other.

**For C++:** this is achieved by publishing/subscribing relative to this node handle that starts the node. The key snippets of code for this are:

.. code-block:: cpp

  // Start the node by initialising a node handle
  ros::NodeHandle nh("~");

Publishing and subscribing are then identical to above, except that you use :code:`nh` instead of :code:`nh_for_group`.


**For Python:** this is achieved by giving the topic name (without any leading backslash) when intialising the publisher/subscriber. The key snippets of code for this are:

.. code-block:: python

  # Initialise a publisher
  self.m_publisher = rospy.Publisher("great_topic", UInt32, queue_size=10, latch=False)


.. code-block:: python

  # Initialise a subscriber
  rospy.Subscriber("great_topic", UInt32, self.subscriberCallback, queue_size=1)



References
**********

The steps detailed on this page are mostly taken from:

* `ROS overview: roscpp Publishers and Subscribers <https://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers>`_
* `ROS overview: rospy Publishers and Subscribers <https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers>`_



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
