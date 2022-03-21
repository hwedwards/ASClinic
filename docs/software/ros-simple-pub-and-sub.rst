.. _ros-simple-pub-and-sub:

Simple ROS Publisher and ROS Subscriber
=======================================

The key ROS concepts in this workflow are generic to ROS, for which the following ROS tutorial pages are the most relevant:

* `Writing a Simple Publisher and Subscriber (C++) <https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`_
* `Writing a Simple Publisher and Subscriber (Python) <https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_
* `Examining the Simple Publisher and Subscriber <https://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber>`_

**Goal of this workflow:** use the templates provided as part of this :code:`asclinic-system` repository to create a publish and subscribe connection between two separate nodes. One node is witten in python and the other node is written in C++, and the message communicated is a custom message.

.. note::

  This workflow is a template for writing a publisher and subscriber, hence for your own implementation you need to replace the names of files and topics with names appropriate to your application.


Create a copy of the C++ template to work with
**********************************************

The intention of the templates provided is that they can be used as a starting point for developing your nodes. Hence the first step is to create a copy of the appropriate template so that we can edit it as part of this workflow.

**Step 1.** Make a copy of the :code:`template_cpp_node_minimal.cpp` file file located at the relative path :code:`catkin_ws/src/asclinic_pkg/src/nodes/`. For this workflow we will name the file :code:`my_publisher.cpp`.

.. note::

  The file name ":code:`my_publisher`" is chosen as an example, replace throughout this workflow with the name of your file.

As this is a C++ node, it needs to be compiled in order to be able to run. Seeing as we made a copy of existing file and gave it a different name, the :code:`my_publisher.cpp` does not appear in the :code:`CMakeLists.txt` and hence will not be compiled by the ROS package :code:`asclinic_pkg`. The following steps adds the new C++ file to the compilation process.

**Step 2.** Open the :code:`CMakeLists.txt` file for editing, it is located at the relative path :code:`catkin_ws/src/asclinic_pkg/src/`.

New C++ files for the ROS package need to be added at three locations within the :code:`CMakeLists.txt` file:

* :code:`add_executable(...)`
* :code:`add_dependencies(...)`
* :code:`target_link_libraries(...)`

As the :code:`CMakeLists.txt` provides the instructions for how to perform the compilation of the ROS package, the ordering of the instructions are important. Hence the easiest way to edit the :code:`CMakeLists.txt` is to follow the pattern of an existing file.

**Step 3.** Insert the following three lines into the :code:`CMakeLists.txt` at the appropriate location to match the instructions already there. Remember to replace :code:`my_publisher` with the name you gave to your file.

* (Step 3a) Insert the following together with the group of :code:`add_executable(...)` instructions:

.. code-block:: bash

	add_executable(my_publisher src/nodes/my_publisher.cpp)

* (Step 3b) Insert the following together with the group of :code:`add_dependencies(...)` instructions:

.. code-block:: bash

	add_dependencies(my_publisher asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

* (Step 3b) Insert the following together with the group of :code:`target_link_libraries(...)` instructions:

.. code-block:: bash

	target_link_libraries(my_publisher ${catkin_LIBRARIES})


.. note::

  If you are copying a different template for your new node, then be sure to review the :code:`CMakeLists.txt` for that template to check whether any flag (or similar) need to be included in these three instruction. For example, a node that uses GPIO (and hence includes the :code:`<gpiod.h>` header file), needs the compilation flag :code:`-lgpiod` at the end of the :code:`target_link_libraries(...)` instruction.


C++ publisher on a custom global node handle
********************************************

**Step 1.** Update the :code:`ros::init` line in the main function to agree with the name of the C++ publisher node, i.e., remove the following line of code from the :code:`my_publisher.cpp` file:

  .. code-block:: cpp

    ros::init(argc, argv, "template_cpp_node_minimal");

  And replace it with the following line:

  .. code-block:: cpp

    ros::init(argc, argv, "my_publisher");


A ROS publisher is created in C++ by calling the :code:`advertise` function on a variable of type :code:`ros::NodeHandle`, which causes the ROS publisher to advertise its topic under the namespace of the node handle used when creating it. Hence, to understand where a message is being published, we must first understand what node handle was used when creating it.

In the template C++ file, the following line of code in the main function creates a :code:`ros::NodeHandle` type variable with the variable name :code:`nodeHandle`:

.. code-block:: cpp

  ros::NodeHandle nodeHandle("~");

The argument :code:`"~"` means that the namespace of the :code:`nodeHandle` variable is the namespace of the node. And the namespace of the node depends on how it was launched. For example, if the node is launch into a group with the namespace :code:`pub_and_sub_workflow`, then the namespace of the :code:`nodeHandle` variable will be

.. code-block:: bash

  /pub_and_sub_workflow/my_publisher

Note importantly that the name of the node, i.e., :code:`my_publisher`, is appended relative to the namespace of the launch group. This can be initially cumbersome to work with because when trying to point to this namespace from another node, for example for a :code:`my_subscriber` node, then the :code:`nodeHandle("~")` variable of that subscriber node will point to a different namespace.

Namespaces can be thought of as a folder path, an analogy that is hopefully easy to follow because the parts of the namespace are separted by :code`/` characters. Hence one easy way to initially manage your namespace is to use a global namespace. See the `ROS wiki node handles overview <http://wiki.ros.org/roscpp/Overview/NodeHandles>`_ where it states: *"This is generally discouraged, [...]. There are times, however, when using global names in code can be useful."*


**Step 2.** Add the following line of code to the main function of :code:`my_publisher.cpp` to create a :code:`ros::NodeHandle` type variable with the variable name :code:`node_handle_for_global` that points to the namespace :code:`/my_global_namespace`:

.. code-block:: cpp

  ros::NodeHandle node_handle_for_global("/my_global_namespace");


**Step 3.** Remove the following line of code from the main function:

  .. code-block:: cpp

    m_template_publisher = nodeHandle.advertise<std_msgs::UInt32>("great_topic", 10, false);

  And replace it with the following line of code that publishes a topic to the global namespace:

  .. code-block:: cpp

    m_template_publisher = node_handle_for_global.advertise<std_msgs::UInt32>("great_topic", 10, false);


**Step 4.** Remove the parts of the code that related to the subscriber because :code:`my_publisher.cpp` is intended to be purely a publisher. Therefore, remove the creation of the variable :code:`template_subscriber` from the main function, and completely remove the function :code:`templateSubscriberCallback`.


**Step 5.** Compile the package so that these changes to :code:`my_publisher.cpp` are compile, i.e., change directory to :code:`catkin_ws` and run the command :code:`catkin_make`.


**Step 6.** Add a new launch file to the :code:`launch` folder with the filename :code:`pub_and_sub_workflow.launch`, i.e., in the relative path :code:`catkin_ws/src/asclinic_pkg/launch/` with the following contents:

.. code-block:: cpp

  <launch>
    <group ns="pub_and_sub_workflow">
      <node
        pkg    = "asclinic_pkg"
        name   = "my_publisher"
        output = "screen"
        type   = "my_publisher"
      />
    </group>
  </launch>

**Step 7.** Launch the node using the command:

  .. code-block:: cpp

    roslaunch asclinic_pkg pub_and_sub_workflow.launch


**Step 8.** Open a separate terminal and check that the topic is published by running the command:

  .. code-block:: cpp

    rostopic list

  This output displayed should include:

  .. code-block:: cpp

    /my_global_namespace/great_topic

  To display message being published on this topic, use the command:

  .. code-block:: cpp

    rostopic echo /my_global_namespace/great_topic


Create a copy of the Python template to work with
*************************************************

**Step 1.** Make a copy of the :code:`template_py_node_minimal.py` file file located at the relative path :code:`catkin_ws/src/asclinic_pkg/src/nodes/`. For this workflow we will name the file :code:`my_subscriber.py`.

.. note::

  The file name ":code:`my_subscriber`" is chosen as an example, replace throughout this workflow with the name of your file.

As this is a Python node, it does **not** need to be compiled in order to be able to run, but you do need to change the contents on the file so that you can meaningfully launch it.


Python subscriber on a custom node handle
*****************************************

**Step 1.** Update the code related to :code:`rospy.init_node(node_name)` in the :code:`__main__` function to agree with the name of the python subscriber node, i.e., remove the following line of code from the :code:`my_subscriber.py` file:

  .. code-block:: python

    node_name = "my_subscriber"

  And replace it with the following line:

  .. code-block:: python

    node_name = "my_subscriber"


We now need to make the subscriber in this python node subscribe to the topic from the C++ publisher described above. For the python syntax of subscribe, there is not the same notion of a node handle as in the C++ syntax. In python you simply specify the topic name to start with a :code:`/` and it will be a global name.

**Step 2.** Remove the following line of code from the :code:`__init__` function:

  .. code-block:: python

    rospy.Subscriber(node_name+"/template_topic", UInt32, self.templateSubscriberCallback)

  And replace it with the following line of code that publishes a topic to the global namespace:

  .. code-block:: python

    rospy.Subscriber("/my_global_namespace"+"/great_topic", UInt32, self.templateSubscriberCallback)

**Step 3.** Remove the parts of the code that related to the publisher because :code:`my_subscriber.py` is intended to be purely a publisher. Therefore, remove the creation of the variable :code:`self.template_publisher` and remove the creation of the timer :code:`self.Timer(...)` from the :code:`__init__` function, and completely remove the function :code:`def timerCallbackForPublishing(...)`.

**Step 4.** Update the launch file :code:`pub_and_sub_workflow.launch` to also launch the python code, i.e., update the full contents to be:

.. code-block:: cpp

  <launch>
    <group ns="pub_and_sub_workflow">
      <node
        pkg    = "asclinic_pkg"
        name   = "my_publisher"
        output = "screen"
        type   = "my_publisher"
      />
      <node
        pkg    = "asclinic_pkg"
        name   = "my_subscriber"
        output = "screen"
        type   = "my_subscriber.py"
      />
    </group>
  </launch>

**Step 7.** Launch the nodes using the command:

  .. code-block:: cpp

    roslaunch asclinic_pkg pub_and_sub_workflow.launch

  You should now see the :code:`my_subscriber` node display the message data in the terminal window.


C++ publisher on a custom message type
**************************************

Coming soon.


Python publisher on a custom message type
*****************************************

Coming soon.


C++ publisher on a group namespace node handle
**********************************************

As your ROS ecosystem grows and you become more proficient with ROS, use will inevitably run into situations where using global namespaces becomes tedious or untenable. A useful level of generalisation is to launch nodes in groups with a namespace for the group specified in the launch file, and then use a node handle for the group namespace to publish and subscribe to topics.

The following two lines of code create a node handle to the groups namespace:

.. code-block:: cpp

  std::string namespace_for_group = ros::this_node::getNamespace();
  ros::NodeHandle node_handle_for_group(m_namespace);

The first line gets a string to the namespace of the node, which will hence be the namespace of the group in which the node was launched. The second line creates a :code:`ros::NodeHandle` type variable with the variable name :code:`node_handle_for_group`. You can subsequently use :code:`node_handle_for_group.advertise(...)` and :code:`node_handle_for_group.subscribe(...)` to publish and subscribe to topics within this namespace.
