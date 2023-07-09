.. _building-block-image-streaming-with-ros:

Stream ROS Images to web browser
================================

Background
**********

There are many possible packages, pipelines, protocols, and architectures for streaming images and video over a network (motion, gstreamer, janus, webrtc; to name just a few). Each option provides a different trade-off between the many aspects to be considered when streaming images and/or video, for example:

* Ease of installation.
* Ease and flexibility of usage.
* Ability to multicast.
* Automatically adapts quality to available bandwidth.
* Server-side load.
* ... and more.

The `Robot Web Tools <https://robotwebtools.github.io>`_ initiative provides a number of packages that make it quick and easy to send and receive ROS messages from the web browser of any device that is on the same network as the robot that is running ROS. The key advantage of these tools is that you can send and receive ROS messages from any device **without needing to install ROS on that device!** Thus the Robot Web Tools provide the possibility to monitor and command you robot from the comfort of your smartphone (though you may need to do some html and JavaScript programming to customise things to your use case).

For the goal of viewing ROS *Image* messages from the :code:`sensor_msgs` package, the Robot Web Tools initiative provides the `web_video_server package <https://github.com/RobotWebTools/web_video_server>`_ as a plug-and-play solution. For completeness of referencing:

* `ROS wiki page for the web_video_server package <https://wiki.ros.org/web_video_server>`_
* `GitHub repository for the web_video_server package <https://github.com/RobotWebTools/web_video_server>`_
* `ROS wiki page for the sensor_msgs package <http://wiki.ros.org/sensor_msgs>`_
* `Definition of the Image message <http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html>`_
* `ROS Tutorial page for converting between OpenCV and ROS Image formats <http://wiki.ros.org/cv_bridge/Tutorials>`_



Install :code:`web_video_server` package
****************************************

The :code:`web_video_server` package is installed on Ubuntu using the following command:

.. code-block:: bash

  sudo apt install ros-noetic-web-video-server

After installation, the :code:`web_video_server` package and node is available from any terminal that sources the ROS setup.bash.



Launch the :code:`web_video_server` node
****************************************

To stream images from your robot to any web browser on the network, the only thing you need to do is run the :code:`web_video_server` node on your robot:

.. code-block:: bash

  rosrun web_video_server web_video_server

This runs a HTTP server and a ROS node with the default configurations of:

* :code:`address` (string, default: 0.0.0.0) This is the address where the server listens for HTTP requests.
* :code:`port` (integer, default: 8080) This is the port number where the server listens for HTTP requests.
* :code:`server_threads` (integer, default: 1) The number of threads used to serve HTTP requests (these are not used for actual image streams).
* :code:`ros_threads` (integer, default: 2) The number of threads running ROS spinners. These threads are used to do the actual image encoding. These threads are shared between all current streams so the number does not need to match the number of connections.

You can configure these parameters of the :code:`web_video_server` node by using a launch file to launch the node:

.. code-block:: bash
  
  <launch>
    <!-- LAUNCH A WEB VIDEO SERVER NODE -->
    <node
      pkg    = "web_video_server"
      name   = "web_video_server"
      output = "screen"
      type   = "web_video_server"
      >
      <param name="port" type="int" value="8080"/>
      <param name="server_threads" type="int" value="1"/>
      <param name="ros_threads" type="int" value="3"/>
    </node>
  </launch>



View the steam of images from a web browser
*******************************************

Once the :code:`web_video_server` node is launched, go to the web browser on any device that is on the same network as your robot, and visit the following address:

.. code-block:: bash

  http://<ip_address>:<port_number>/

where:

* :code:`<ip_address>` is replaced with the IP address or hostname of your robot, and
* :code:`<port_number>` is replaced with the port number that you configured when launching the web video server.

For example, if the IP address of your robot is :code:`192.168.1.42` and you are using the default port, then the web address is:

.. code-block:: bash

  http://192.168.1.42:8080/

This displays a webpage that lists the currently available ROS topics that have *Image* as their message type. Each topic is hyperlinked allowing you to open the stream or to grab a single snapshot on that topic.

As per the `ROS wiki page for web_video_server <https://wiki.ros.org/web_video_server>`_, you can configure some attributes of the stream using parameters in the HTTP address. For example, to stream with a quality of 50%, use the following address:

.. code-block:: bash

  http://<ip_address>:<port_number>/stream?topic=<topic_name>&quality=50

where :code:`<topic_name>` is replaced with the name of the topic you wish to stream, which for example might be: :code:`/asc/camera_image`

To further reduce bandwidth by reducing image size, specify the :code:`width` and :code:`height` in the stream address, for example:

.. code-block:: bash

  http://<ip_address>:<port_number>/stream?topic=<topic_name>&quality=50&width=960&height=540


Configure your :code:`package.xml` and :code:`CMakeLists.txt` accordingly
*************************************************************************

If you are using OpenCV to capture and process images before publishing them as ROS messages, then you need to configure your ROS package for the libaries that convert from OpenCV format to ROS Image format.

Ensure that the :code:`package.xml` of your ROS package contains the following:

.. code-block:: bash

  <build_depend>sensor_msgs</build_depend>
  <build_depend>cv_bridge</build_depend>

  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>cv_bridge</build_export_depend>

  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>

Ensure that the :code:`CMakeLists.txt` of your ROS package contains the lines that are highlighted in the following:

* At the top of the file:

  .. code-block:: bash
    :emphasize-lines: 7,8

    find_package(catkin REQUIRED COMPONENTS
      message_generation
      roscpp
      rospy
      std_msgs
      geometry_msgs
      sensor_msgs
      cv_bridge
      genmsg
      roslib
    )

* Under the heading "Declare ROS messages, services and actions":

  .. code-block:: bash
    :emphasize-lines: 5

    generate_messages(
      DEPENDENCIES
      std_msgs
      geometry_msgs
      sensor_msgs
    )

* Under the heading "catkin specific configuration":

  .. code-block:: bash
    :emphasize-lines: 7,8

    catkin_package(
      CATKIN_DEPENDS
      roscpp
      rospy
      std_msgs
      geometry_msgs
      sensor_msgs
      cv_bridge
      roslib
    )

Ensure that your Image message uses an appropriate encoding
***********************************************************

For converting OpenCV images to ROS Images, the key lines of code in a Python node are as follows.

* Import the message type, OpenCV, and the package that bridges from OpenCV to ROS:

  .. code-block:: python

    # Import the message type
    from sensor_msgs.msg import Image
    # Import opencv
    import cv2
    # Package to convert between ROS and OpenCV Images
    from cv_bridge import CvBridge, CvBridgeError

* Initialise the ROS publisher for *Image* type messages, and the :code:`CvBridge` object for converting images:

  .. code-block:: python

    # Initialise a publisher for the images
    self.image_publisher = rospy.Publisher("/asc"+"/camera_image", Image, queue_size=10)
    # Initialise video capture from the camera
    self.cam=cv2.VideoCapture(0)
    # Initialise the OpenCV<->ROS bridge
    self.cv_bridge = CvBridge()

* Capture an OpenCV format of image from the camera:

  .. code-block:: python

    # Read the camera frame
    return_flag , current_frame = self.cam.read()

* Convert the camera frame to ROS *Image* format (with :code:`bgr8` encoding) and publish it:

  .. code-block:: python

    # Publish the camera frame
    try:
        self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame, "bgr8"))
    except CvBridgeError as cv_bridge_err:
        print(cv_bridge_err)

  **Note that** this conversion example fails if :code:`current_frame` is a grayscale image because the example is requesting conversion to a colour format. The options for the encoding string is:

    * :code:`mono8`: :code:`CV_8UC1`, grayscale image
    * :code:`mono16`: :code:`CV_16UC1`, 16-bit grayscale image
    * :code:`bgr8`: :code:`CV_8UC3`, color image with blue-green-red color order
    * :code:`rgb8`: :code:`CV_8UC3`, color image with red-green-blue color order
    * :code:`bgra8`: :code:`CV_8UC4`, BGR color image with an alpha channel
    * :code:`rgba8`: :code:`CV_8UC4`, RGB color image with an alpha channel

    Note that **mono8** and **bgr8** are the two image encodings expected by **most OpenCV functions**.

.. note::

  For more details, explanations, and example code for converting OpenCV images to ROS image, see this `cv_bridge tutorial for python <http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython>`_



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
