.. _ros:

ROBOT OPERATING SYSTEMS (ROS)
=============================

.. toctree::
  :maxdepth: 2
  :hidden:

  ros_in_words
  ros_code_from_a_clean_slate
  ros_code_first_robotic_system
  ros_cmd_line
  


ROS (the robot operating system) is a large topic with many plausible levels-of-detail and plausible presentation/tutorial methods. This wiki in separated into the following sub-sections that increase in complexity and level-of-detail.

  #. :ref:`ros-in-words`: provides an introduction to the core concepts and key elements of ROS within involving any programming. This level-of-detail should be enough to design a first draft of a ROS architecture for how you bring your robotics project together using ROS.


  #. :ref:`ros-code-from-a-clean-slate`: provides step-by-step tutorials for how the core concepts and key elements of ROS materialise through code (C++ and Python) and command line tools. This section will likely feel quite boring (apologies for that) because no exciting robotics behaviour is achieved. But remember that **this section contains the basic building blocks needed to do everything exciting in the next section**.


  #. :ref:`ros-code-first-robotic-system`: provides step-by-step tutorials for how to put together the building blocks from the previous section to create a first robotic system.


  #. :ref:`ros-cmd-line`: provides a list and brief explanation for the essential command line tools.



..
  .. literalinclude:: ros.rst
    :language: RST
    :emphasize-lines: 12,15-18
    :linenos:



..
  NOTES FOR WHAT STILL TO INCLUDE:
  
  - Who uses ROS (lots of example, links, and videos, https://www.designnews.com/automation-motion-control/10-robot-companies-you-should-know-use-ros)
  - Limitations of ROS (latency) (topic transport: http://wiki.ros.org/Topics)
  - What is ROS 2
  - Installation of ROS (simply link to instructions) but make the Ubuntu ROS version clear
  - What programming language does ROS use (including note about python version)
  - Structure of folder in ROS (this is key to how the toolchain works)
  - Describe how to have multiple of the same asclinic_pkg on the same computer.
  - Put it all together, use this as a hint to draw a nice feedback loop with graphviz:
    - https://stackoverflow.com/questions/54724783/graphviz-drawing-nodes-in-given-order-to-correctly-draw-tree