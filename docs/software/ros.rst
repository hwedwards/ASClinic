.. _ros:

ROBOT OPERATING SYSTEMS (ROS)
=============================

.. toctree::
  :maxdepth: 2
  :hidden:

  ros_what
  ros_why
  ros_key_elements
  ros-simple-pub-and-sub
  ros_cmd_line
  


.. admonition:: Contents of this part posed as questions

  #. :ref:`ros-what` Explain at a high-level.
  #. :ref:`ros-why` Explain why the features of ROS are considered useful for developing a robot.
  #. Explain each of :ref:`ros-key-elements`, i.e., explain each of the following:

     - Nodes
     - Topics
     - Messages
     - Publishing and subscribing
     - Services

     Additionally explain how these elements are used together, drawing an example to support your explanation.
  
  #. What is the code required to create a :ref:`ros-simple-pub-and-sub`, both in C++ and in Python.
  #. List and explain some of the :ref:`ros-cmd-line` essentials. As part of this, explain the difference between using :code:`rosrun` and :code:`roslaunch` to start up nodes.





..
  NOTES FOR WHAT TO INCLUDE:
  - What is ROS (service oriented architecture)
  - Why use ROS (it is the functionality you would anyway create if you started from scratch)
  - Is it a blessing or a necessary evil.
  - Who uses ROS (lots of example, links, and videos, https://www.designnews.com/automation-motion-control/10-robot-companies-you-should-know-use-ros)
  - Limitations of ROS (latency) (topic transport: http://wiki.ros.org/Topics)
  - What is ROS 2
  - Installation of ROS (simply link to instructions) but make the Ubuntu ROS version clear
  - What programming language does ROS use (including note about python version)
  - Structure of folder in ROS (this is key to how the toolchain works)
  - catkin_make produces the build and devel folders (deleting these can make a difference when there are "strange" errors)
  - To access the auto-completion, you need to course setup.bash EVERY time you open a connection (add details for adding to .bashrc)
  - Describe what catkin_make does (looked into "_pkg" folder; then followed instructions in the CMakeList.txt; which gives instructions to compile the C++ files in the folder /src/nodes; then the excitable files are stored in the build and devel folder.)
  - Python files can be launched anytime.
  - Describe how to have multiple of the same asclinic_pkg on the same computer.
  - How to launch (roslaunch versus rescore + rosrun)
  - Step-by-step instructions for adding a python and C++ ROS node the exemplifies custom messages in both directions, services in both directions, launching, and terminal interrogation. Use a robot function to motivate.