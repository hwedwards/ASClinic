.. _ros-define-and-use-custom-message-types:

Define and use custom message types (C++ and Python)
====================================================

Requirements for this page:

  * ROS is installed on the computer you are working on (be that working "directly" on the computer or remotely connected to it).
  * A catkin workspace and ROS package initialised as per :ref:`ros-create-catkin-ws-and-package`
  * The publisher and subscriber nodes of :ref:`ros-code-pub-and-sub-simple`

.. contents:: Contents of this page
   :local:
   :backlinks: none
   :depth: 2



TL;DR
*****

Of course you did already read everything below, and the following TL;DR summary is just for convenience when you return to remind yourself of a small detail:

Define the custom message type with a :code:`*.msg` file at this location:

.. code-block:: bash

  nano catkin_ws/src/my_robotics_pkg/msg/MyCustomMessageType.msg

The file contents defines the message structure, for example:

.. code-block:: bash

  bool isValid
  uint32 restartsRemaining
  int32 encoderCounts
  float32 distanceToStart
  float64 distanceToEnd
  string taskDescription
  float64[] distancesToOtherRobots

Add to the :code:`CMakeLists.txt` the directive to compile the custom message:

.. code-block:: bash

  add_message_files(
    FILES
    MyCustomMessageType.msg
  )

An then compile the message type into a header, i.e., :code:`catkin_make`

Import the custom message type into:

* **For C++:**

.. code-block:: cpp

  // Include the custom message types
  #include "my_robotics_pkg/MyCustomMessageType.h"

* **For Python:**

.. code-block:: python

  # Import the custom message types
  from my_robotics_pkg.msg import MyCustomMessageType


High-level description (BOTH)
*****************************

Custom messages are essentially structs (i.e., dictionaries with key-value pairs) that you define with a short :code:`*.msg`. The fields of a custom message can be any of the standard message types or even other custom message types, i.e., you can nest the definitions of your custom messages. Field of a custom message can be a vector of known or unknown length.

Once you have define your custom message type via a :code:`*.msg` file, it must be compiled to make the definition available. And then you can use it like any other message type.



Define the custom message, in a :code:`*.msg` file
**************************************************

The name of this :code:`*.msg` file is the name of the custom message type when using it in your C++ and Python code.

The recommended location to keep you custom message definition is within a :code:`msg` folder with your ROS package. For the purpose of this tutorial, we create a custom message named :code:`MyCustomMessageType` by creating the following file:

.. code-block:: bash

  nano catkin_ws/src/my_robotics_pkg/msg/MyCustomMessageType.msg

And giving the file the following contents:

.. code-block:: bash

  bool isValid
  uint32 restartsRemaining
  int32 encoderCounts
  float32 distanceToStart
  float64 distanceToEnd
  string taskDescription
  float64[] distancesToOtherRobots

The names given to each field are just for a bit of "fun". The key details are the conventions for the primitive types, i.e.:

* :code:`bool`
* :code:`uint8`, :code:`uint16`, :code:`uint32`
* :code:`int8`, :code:`int16`, :code:`int32`
* :code:`float32`, :code:`float64`
* :code:`string`

Any of these can be made an array of **unknown length** by adding :code:`[]`

Any of these can be made an array of **known length** by adding :code:`[42]`, where :code:`42` is replaced by the desired length.



Compile the custom message type
*******************************

.. important::

  This step applied also for using the custom message in a Python script. Even though Python scripts do not need to be compile, the custom message needs to be compiled so that it is available for import to your Python scripts.

In order to compile your custom message type, you need to add the directive to the :code:`CMakeLists.txt` file instructing each custom message to be compile.

Open the :code:`CMakeLists.txt`:

.. code-block:: bash

  nano catkin_ws/src/my_robotics_pkg/CMakeLists.txt

Locate the following comments that are part of the automatically generated file, and just below the comments add the directive to compile your custom message type:

.. code-block:: bash
  :emphasize-lines: 8-11

  ## Generate messages in the 'msg' folder
  # add_message_files(
  #   FILES
  #   Message1.msg
  #   Message2.msg
  # )

  add_message_files(
    FILES
    MyCustomMessageType.msg
  )

Now compile in the usual fashion using :code:`catkin_make`:

.. code-block::

  cd catkin_ws
  catkin_make



Import the custom message definition into your C++ and Python code
*********************************************************************

As with any message type that you use within the code of your C++ or Python nodes, you need to include/import the definition of the message. As the custom message is defined as part of your ROS package, it is included/imported from this as the source.

**For C++:**

.. code-block:: cpp

  // Include the custom message types
  #include "my_robotics_pkg/MyCustomMessageType.h"

**For Python:**

.. code-block:: python

  # Import the custom message types
  from my_robotics_pkg.msg import MyCustomMessageType

.. note::

  :code:`my_robotics_pkg` should be replaced by the name of your ROS package.

  :code:`MyCustomMessageType` should be replaced by the name of your custom message type.



Using custom messages within C++
********************************

In the initialisation of publishers or in subscriber function argument, you simply replace something like :code:`std_msgs::UInt32` with :code:`my_robotics_pkg::MyCustomMessageType`

As a more concrete example, a publisher initialisation becomes:

.. code-block:: cpp
  :emphasize-lines: 4

  // Initialise a publisher relative to the group namespace
  uint32_t queue_size = 10;
  bool should_latch = false;
  m_publisher = nh_for_group.advertise<my_robotics_pkg::MyCustomMessageType>("great_custom_topic", queue_size, should_latch);

And a subscriber function argument becomes:

.. code-block:: cpp
  :emphasize-lines: 2

  // Implement the subscriber callback function
  void subscriberCallback(const my_robotics_pkg::MyCustomMessageType& msg)
  {
      // Contents of function...
  }

To construct a custom message you simply use the fields from its definition. Vector fields of the message are handled as C++ :code:`std::vector` type, and hence you use :code:`push_back()` to add data to such fields. And then you publish the message in the normal fashion.

.. code-block:: cpp

  // Implement the timer callback function
  void timerCallback(const ros::TimerEvent&)
  {
      // Initialise an empty message of the custom type
      my_robotics_pkg::MyCustomMessageType msg;

      // Fill in the fields of the message
      msg.isValid = true;
      msg.restartsRemaining = 42;
      msg.distanceToStart = 4.2;
      msg.distanceToEnd = 42.42;
      msg.taskDescription = "Staying alive";

      // "Push back" data into the vector field
      msg.distancesToOtherRobots.push_back(1.1);
      msg.distancesToOtherRobots.push_back(2.3);
      msg.distancesToOtherRobots.push_back(2.7);

      // Publish the message
      m_publisher.publish(msg);
  }

Extracting data within a subscriber callback in essentially the same. For vectors it is recommended to use :code:`size()` to get the number of elements in the vector, and then to access those elements using :code:`.at()`.

.. code-block:: cpp

  // Implement the subscriber callback function
  void subscriberCallback(const my_robotics_pkg::MyCustomMessageType& msg)
  {
      // Extract the data from the message
      bool is_valid = msg.isValid;
      uint restarts_remaining = msg.restartsRemaining;
      // and so on for the other fields.

      // Print the elements of the vector
      int vector_size = size(msg.distancesToOtherRobots);
      for (int i=0; i<vector_size; i++)
      {
          ROS_INFO_STREAM("[SUBSCRIBER CPP NODE] distancesToOtherRobots[" << i << "] = " << distancesToOtherRobots.at(i));
      }
  }


Using custom messages within Python
***********************************

In the initialisation of publishers or subscribers, you simply replace something like :code:`UInt32` with :code:`MyCustomMessageType`

As a more concrete example, a publisher initialisation becomes:

.. code-block:: python
  :emphasize-lines: 5

  class PublisherPyNode:

    def __init__(self):
        # Initialise a publisher
        self.m_publisher = rospy.Publisher(this_nodes_namespace + "/great_custom_topic", MyCustomMessageType, queue_size=10, latch=False)

And a subscriber initialisation becomes:

.. code-block:: python
  :emphasize-lines: 5

  class SubscriberPyNode:

    def __init__(self):
        # Initialise a subscriber
        rospy.Subscriber(this_nodes_namespace + "/great_custom_topic", UInt32, self.subscriberCallback, queue_size=1)

To construct a custom message you simply use the fields from its definition. And then you publish the message in the normal fashion.

.. code-block:: python

  # Implement the timer callback
  def timerCallback(self, event):
      # Initialise an empty message of the custom type
      msg = MyCustomMessageType();

      # Fill in the fields of the message
      msg.isValid = True
      msg.restartsRemaining = 42
      msg.distanceToStart = 4.2
      msg.distanceToEnd = 42.42
      msg.taskDescription = "Staying alive"

      # Publish a message
      self.m_publisher.publish(msg)
  }

Extracting data within a subscriber callback in essentially the same.

.. code-block:: python

  # Implement the subscriber callback
  def subscriberCallback(self, msg):
      # Extract the data from the message
      is_valid = msg.isValid
      restarts_remaining = msg.restartsRemaining
      # and so on for the other fields.

      # Display the data
      rospy.loginfo("[SUBSCRIBER PY  NODE] received custome message with isValid = " + str(is_valid)



References
**********

The steps detailed on this page are mostly taken from:

* `ROS tutorial: Creating Messages and Services <https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv>`_



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
