.. _building-blocks-for-autonomous-systems:

AUTONOMOUS SYSTEM BUILDING BLOCKS
=================================

.. toctree::
  :maxdepth: 2
  :hidden:
  :numbered:

  building_block_rplidar
  building_block_usb_camera_settings
  building_block_camera_calibration
  building_block_aruco_detection
  building_block_image_streaming_with_ros
  building_block_i2c
  building_block_i2c_pololu_smc_g2
  building_block_i2c_servo_driver_pca9685
  building_block_gpio_single_pin
  building_block_gpio_encoder_counting
  
  
  
  

Many autonomous ground vehicles (aka. AGVs) require similar building blocks (aka. sub-systems) to operate. Reliable autonomous operation requires detailed engineering design and implementation for both: the separate building block (aka. sub-systems); and the interaction of all those building block that together create the overall autonomous system behaviour.

This section provides step-by-step guides for some of the key hardware-software building blocks for a differential drive robot. The topics covered are:

* :ref:`Obtaining Lidar scan data from an RPLidar rotating time-of-flight distance sensor (model A1, A2, or A3). <building-block-rplidar>`

* :ref:`Interrogating and adjusting (web) camera setting via command line. <building-block-usb-camera-settings>`

* :ref:`Recording images from a (web) camera, and subsequently using these for calibrating the intrinsic parameters. <building-block-image-streaming-with-ros>`

* :ref:`Detecting ArUco fiducial markers in (web) camera images, and estimating the pose of the marker relative to the camera <building-block-aruco-detection>`.

* :ref:`Steam ROS Image type messages to be viewed from any browser on the network <building-block-image-streaming-with-ros>`.

* :ref:`Setting up an I2C communication bus. <building-block-i2c>`

* :ref:`Commanding the drive-wheel motors via I2C communication. <building-block-i2c-pololu-smc-g2>`

* :ref:`Commanding the servos via I2C communication. <building-block-i2c-servo-driver-pca9685>`

* :ref:`Reading data via a single GPIO pin. <building-block-gpio-single-pin>`

* :ref:`Reading and counting encoder data via GPIO pins. <building-block-gpio-encoder-counting>`


.. important::

  ROS code is part of the step-by-step guides for the building blocks. However, when developing and implementing each building block separately, ROS is not required.

  When combining multiple building blocks into an autonomous system, then ROS is the glue that binds everything together. ROS makes it easy to send messages between that building blocks, for example: sending a message from the Lidar block to the localisation block with the Lidar scan data as the contents of the message.

  Hence, ROS code is part of the step-by-step guides for the building blocks. See the :ref:`ros-in-words` page for more details about ROS.



As a friendly reminder of the many sub-systems required for an AGV to rover about autonomously, these are some of the building blocks not covered by this section:

* Odometry computations.
* Localisation by filtering together multiple measurement streams.
* Path planning and finite-state-machine (FSM) cognition.
* Trajectory generation and trajectory tracking control.
* ... more?



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
