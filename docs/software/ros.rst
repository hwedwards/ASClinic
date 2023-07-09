.. _ros:

ROBOT OPERATING SYSTEMS (ROS)
=============================

.. toctree::
  :maxdepth: 2
  :hidden:

  ros_in_words
  ros_code_from_a_clean_slate
  ros_cmd_line
  


ROS (the robot operating system) is a large topic with many plausible levels-of-detail and plausible presentation/tutorial methods. This wiki in separated into the following sub-sections that increase in complexity and level-of-detail.

  #. :ref:`ros-in-words`: provides an introduction to the core concepts and key elements of ROS within involving any programming. This level-of-detail should be enough to design a first draft of a ROS architecture for how you bring your robotics project together using ROS.


  #. :ref:`ros-code-from-a-clean-slate`: provides step-by-step tutorials for how the core concepts and key elements of ROS materialise through code (C++ and Python) and command line tools.


  #. :ref:`ros-cmd-line`: provides a list and brief explanation for the essential command line tools.


.. important::

  Section 2 here (i.e., :ref:`ros-code-from-a-clean-slate`) is by definition rather "boring" because no exciting robotics behaviour is demonstrated.

  To get straight into building you robotic system software, you can skip to the section of the wiki titled (i.e., :ref:`building-blocks-for-autonomous-systems`), which jumps straight into interfacing with the actuators and sensors on your robot. From there you are referred back to specific parts of Section 2 here with motivation for why the "boring" parts of ROS upskilling are so important.

  Section 1 (i.e., :ref:`ros-in-words`) is beneficial for those with beginner and intermediate ROS skills.



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