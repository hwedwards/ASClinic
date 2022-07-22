.. _workflow_rplidar:

RPLidar Data
============


Install and compile the RPLidar ROS package
*******************************************

The installation process is described here: :ref:`install_rplidar_for_ros`

This process clones the RPLidar ROS package git repository into the :code:`catkin_ws/src/` of your git repository. Hence the nodes of the RPLidar ROS package are available after a :code:`catkin_make`, i.e.,:

.. code-block:: bash

  cd ~/asclinic-system/catkin_ws
  catkin_make



Launch the RPLidar node
***********************

The RPLidar ROS package provide a node named :code:`rplidarNode` that read data from the RPLidar device and publishes it to the topic :code:`/scan`. As the RPLidar ROS package is part of your catkin workspace, you can launch an :code:`rplidarNode` in the same way you would launch any other node. The following is a complete launch file for launching the :code:`rplidarNode` into the group namespace :code:`/asc`:

.. code-block:: html

  <launch>

    <group ns="asc">
      <!-- LAUNCH A RPLIDAR NODE -->
      <!-- Note: for model A1/A2 use baudrate 115200 -->
      <!--       for model A3    use baudrate 256000 -->
      <node
        pkg    = "rplidar_ros"
        name   = "rplidarNode"
        output = "screen"
        type   = "rplidarNode"
        >
        <param name="serial_port" type="string" value="/dev/rplidar"/>
        <param name="serial_baudrate" type="int" value="115200"/>
        <param name="frame_id" type="string" value="laser"/>
        <param name="inverted" type="bool" value="false"/>
        <param name="angle_compensate" type="bool" value="true"/>
      </node>
    </group>

  </launch>

For example, you can place this launch file at the location:

.. code-block::

  ~/asclinic-system/catkin_ws/src/asclinic_pkg/launch/rplidar.launch

And then launch it with the following command:

.. code-block:: bash

  roslaunch asclinic_pkg rplidar.launch

.. note::

  As launch file uses the group namespace :code:`/asc`, the data published to by the :code:`rplidarNode` is available on the topic :code:`/asc/scan`



Format of the RPLidar scan data
********************************

The :code:`rplidarNode` publishes one message of type :code:`sensor_msgs::LaserScan` for every complete rotation of the Lidar device.
The `definition of the LaserScan message type <https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html>`_ shows that it contains the following properties:

* :code:`angle_min` - this is the angle at the start of the scan, i.e., the angle of the first measurement in the :code:`ranges` property. In units of [radians].

* :code:`angle_max` - this is the angle at the end of the scan, i.e., the angle of the last measurement in the :code:`ranges` property. In units of [radians].

* :code:`angle_increment` - this is the angular distance between measurements.  In units of [radians].

* :code:`time_increment` - this is the time between measurements. In units of [seconds].

* :code:`scan_time` - this is the time between scans, i.e., the time between the first measurement of this scan and the first measurement of the previous scan.  In units of [seconds].

* :code:`range_min` - this is the minimum range value. In units of [meters].

* :code:`range_max` - this is the maximum range value. In units of [meters].

* :code:`ranges` - this is an array property that contains the range data of each measurement. The length of this array should be number of time that :code:`angle_increment` fits in between :code:`angle_min` and :code:`angle_max`, i.e., the measurement :code:`ranges[0]` was taken at :code:`angle_min`, and the measurements :code:`ranges[i]` were taken at :code:`(angle_min + i*angle_increment)`



.. note::

  Values in the :code:`ranges` array **should be discarded** if they are less than the :code:`range_min` or greater than the :code:`range_max` property.



Subscribe to the RPLidar scan data
**********************************

**Step 1.** include the header that defines the message type.

  **For C++:** the LaserScan include is:

  .. code-block:: cpp

    #include "sensor_msgs/LaserScan.h"

  **For Python:** the LaserScan import is:

  .. code-block:: python

    from sensor_msgs.msg import LaserScan



**Step 2.** Add a ROS subscriber to the :code:`/scan` topic.

  **For C++:** the topic subscriber is:

  .. code-block:: cpp

    // Initialise a node handle to the group namespace
    ros::NodeHandle nh_for_asc_group("/asc);
    // Initialise a subscriber to the RPLidar scan
    ros::Subscriber rplidar_scan_subscriber = nh_for_asc_group.subscribe("scan", 10, laserScanSubscriberCallback);

  **For Python:** the topic subscriber is:

  .. code-block:: python

    # Initialise a subscriber to the RPLidar scan
    rospy.Subscriber("/asc"+"/scan", LaserScan, self.laserScanSubscriberCallback)



**Step 3.** Implement the subscriber callback to process the scan data.

  **For C++:** the subscriber callback is:

  .. code-block:: cpp

    // Respond to subscriber receiving a message
    void laserScanSubscriberCallback(const sensor_msgs::LaserScan& msg)
    {
      ROS_INFO_STREAM("Message received with angle_min = " << msg.angle_min << " [rad], angle_max = " << msg.angle_max << " [rad], range_min = " << msg.range_min << " [m], range_max = " << msg.range_max << " [m]");

      // Now process the msg.ranges data to
      // interpret the robot's surroundings
    }


  **For Python:** the subscriber callback is:

  .. code-block:: python

    # Respond to subscriber receiving a message
    def laserScanSubscriberCallback(self, msg):
        rospy.loginfo("Message receieved with angle_min = " + str(msg.angle_min) + " [rad], angle_max = " + str(msg.angle_max) + " [rad], range_min = " + str(msg.range_min) + " [m], range_max = " + str(msg.range_max) + " [m]")

        # Now process the msg.ranges data to
        # interpret the robot's surroundings


**Step 4.** Ensure that the :code:`sensor_msgs` are made available via the :code:`pacakge.xml` and :code:`CMakeLists.txt` at the path:

  .. code-block::

    ~/asclinic-system/catkin_ws/src/asclinic_pkg/

  Simply copy the syntax and directives used for adding the :code:`std_msgs` to these two files.

  For the :code:`pacakge.xml`, add the following lines:

  .. code-block:: html

    <build_depend>sensor_msgs</build_depend>
    <build_export_depend>sensor_msgs</build_export_depend>
    <exec_depend>sensor_msgs</exec_depend>

  For the :code:`CMakeLists.txt`, add the lines that are highlighted in the following:

  .. code-block::
    :emphasize-lines: 7

    find_package(catkin REQUIRED COMPONENTS
      message_generation
      roscpp
      rospy
      std_msgs
      geometry_msgs
      sensor_msgs
      genmsg
      roslib
    )

  .. code-block::
    :emphasize-lines: 6

    ## Generate added messages and services with any dependencies listed here
    generate_messages(
      DEPENDENCIES
      std_msgs
      geometry_msgs
      sensor_msgs
    )




External links
**************

* `Git repository for the Slamtec RPLidar ROS package <https://github.com/slamtec/rplidar_ros>`_.

* The launch process recommended above is based on the launch file found `here in the RPlidar ROS package <https://github.com/Slamtec/rplidar_ros/tree/master/launch>`_

* `Definition of the LaserScan message type <https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html>`_
