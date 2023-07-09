.. _building-block-aruco-detection:

ArUco Marker Detection
======================


Install OpenCV
**************

Mostly likely this is already installed because it is included in both the :ref:`software-installation-manual` and :ref:`software-installation-script`. If not already installed, follow the :ref:`install_opencv_python` instructions.


Template overview
*****************

To detect markers within images captured by the camera, the template python3 ROS node :code:`aruco_capture.py` is provided. This file is located in the repository at the relative path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/nodes/


The file is extensively commented and the comments serve as the documentation for how to edit the file to implement your use case. **Before running the ArUco detector node,** you should note the following parts of the code, adjusting as required:

* Adjust the parameters at the top of the file to match your use case. Namely:

  * Check that the :code:`USB_CAMERA_DEVICE_NUMBER` agrees with that of the camera plugged into your robot.

  * Adjust the values of :code:`DESIRED_CAMERA_FRAME_HEIGHT` and :code:`DESIRED_CAMERA_FRAME_WIDTH` to be the resolution that you desire.

  * Adjust the value of :code:`DESIRED_CAMERA_FRAME_FPS` to be the frame rate that you desire. **Note that** most cameras do not allow any arbitrary value for the frame per second (fps) property, and generally operate at the fps closest to your request, for example, 5 or 30 fps.

  * Adjust the :code:`MARKER_SIZE` to be the physical size of the ArUco markers placed around the environment.

  * Adjust the path :code:`SAVE_IMAGE_PATH` to be appropriate for your robot and ensure that the path exists.

.. note::

  The units of the :code:`MARKER_SIZE` parameter can be freely chosen, i.e., meters, millimeters, inches. The translation vector returned by the ArUco pose estimate function is expressed in the same units.


.. note::

  The code within :code:`aruco_detecto.py` assumes that all markers within the environment have the same physical :code:`MARKER_SIZE`. If you place markers with different sizes, then you will need to specify the marker size for each marker ID, and adjust the code accordingly.


* The subscriber initialised within the :code:`__init__` function for the topic :code:`request_save_image` provides a convenient way to save images to file so that you can visualise the ArUco detection. If you publish a message to this topic, then the node will save the next camera image to file. You do **NOT** need to create a separate node for publishing a message to this topic. Instead, you can publish to this topic by entering the following in a separate terminal window that is logged in to the robot:

.. code-block:: bash

  rostopic pub /global_namespace/request_save_image std_msgs/UInt32 "data: 1"

* The camera images are captured and processed for ArUco marker detection within the :code:`timerCallbackForPublishing` function. Hence you can adjust the frequency of the camera captures by adjusting the :code:`rospy.Duration` of the timer where it is initialised within the :code:`__init__` function.

  * Alternatively, you could define a ROS rate variable as :code:`rate = rospy.Rate(10)` (where the argument is in hertz), then wrap the contents of the :code:`timerCallbackForPublishing` function within a :code:`while not rospy.is_shutdown():` loop that ends with :code:`rate.sleep()`

* The lines of code for saving images is within the :code:`if (self.should_save_image):` statement. There you see that the file name is index by a number that increments with each imaged saved to file. Hence, within one launch of the node, files will not be over-written. **However**, when you launch the node another time, it will start from index 1 again and hence over-write any previously saved imaged.


The "aruco capture" node is launched by the file :code:`aruco_capture.launch`, and is hence launched by the command:

.. code-block:: bash

  roslaunch asclinic_pkg aruco_capture.launch



Setup steps
***********

Within the :code:`__init__` function of :code:`aruco_detecto.py`, the following important specifications for ArUco marker detection are set:

* The so-called dictionary of ArUco markers to search for:

  .. code-block:: python

    # Get the ArUco dictionary to use
    self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

  This variable is used to specify which unique set of markers the ArUco detection function should search for and made up of:

    * The :code:`4x4` part that specifies that ArUco dictionary of markers with a four-by-four internal grid of black and white square, the pattern of which determines the unique ID of the marker. In other words, this specifies the number of bits available for for encoding unique IDs.

    * The :code:`50` part that specifies the ArUco dictionary is composed of markers with 50 unique IDs.

  .. note::

    As mentioned in the "selecting a dictionary" section of the `OpenCV page on detection of ArUco markers <https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html>`_ - *In general, lower dictionary sizes and higher marker sizes increase the inter-marker distance and vice-versa.*

* The parameters for the ArUco marker detection function:

  .. code-block:: python

    # Create an parameter structure needed for the ArUco detection
    self.aruco_parameters = aruco.DetectorParameters_create()
    # > Specify the parameter for: corner refinement
    self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

  This ArUco detector parameter struct allows you to specify various parameters for how a camera image is processed in order to detector ArUco markers that the image may contain.  The :code:`CORNER_REFINE_SUBPIX` parameter is commonly used to increase the accuracy of the pixel location determined for the corners of a marker.


* The intrinsic parameters of the camera being used:

  .. code-block:: python

    # Specify the intrinsic parameters of the camera
    self.intrinic_camera_matrix = np.array( [[1726,0,1107] , [0,1726,788] , [0,0,1]], dtype=float)
    self.intrinic_camera_distortion  = np.array( [[ 5.5252e-02, -2.3523e-01, -1.0507e-04, -8.9834e-04, 2.4028e-01]], dtype=float)

  The intrinsic camera parameters are required so that the detected ArUco markers (i.e., the bounding box of the markers as it is detected in 2D pixel coordinates) can be meaningfully used to estimate the 3D pose of the marker frame relative to the camera frame.

  .. note::

    The intrinsic camera parameter values shown here and in the "aruco capture" node are hardcoded to exemplify the format. You **must** update these values to be appropriate for the camera you are using, otherwise the ArUco marker pose estimate will be meaningless. For interest, these hardcode intrinsic camera parameter are for a Logitech C922 webcam with focus level at infinity and resolution of 1920x1080.



Detection steps
***************

Within the :code:`timerCallbackForPublishing` function of :code:`aruco_capture.py`, the following are the important steps for detecting ArUco marker detection are with the :code:`current_frame` recorded from the camera:

* Convert the camera image to grayscale

  .. code-block:: python

    current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

* Detect the corners of any ArUco markers within the grayscale image:

  .. code-block:: python

    aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = aruco.detectMarkers(current_frame_gray, self.aruco_dict, parameters=self.aruco_parameters)

  Where the ArUco marker dictionary and the detection parameters (e.g., sub-pixel corner refinement) specified during the setup step are relevant for this detection step.

* Estimate the pose of a particular ArUco marker relative to the camera frame:

  .. code-block:: python

    this_rvec_estimate, this_tvec_estimate, _objPoints = aruco.estimatePoseSingleMarkers(corners_of_this_marker, MARKER_SIZE, self.intrinic_camera_matrix, self.intrinic_camera_distortion)

  Where the ArUco :code:`MARKER_SIZE` and the intrinsic camera parameters specified during the setup step are relevant for accurate pose estimation.

* Extract the rotation matrix and translation vector for this particular ArUco marker relative to the camera frame:

  .. code-block:: python

    rvec = rvec[0]
    tvec = tvec[0]
    # Compute the rotation matrix from the rvec using the Rodrigues
    Rmat = cv2.Rodrigues(rvec)

  * :code:`tvec` is a vector of length 3 expressing the (x,y,z) coordinates of the marker's center in the coordinate frame of the camera.
  * :code:`rvec` is a vector of length 3 expressing the rotation of the marker's frame relative to the frame of the camera. This vector is an "axis angle" representation of the rotation following the Rogrigues convention.
  * :code:`Rmat` is a 3x3 rotation matrix that is constructed from the :code:`rvec` using the :code:`cv2.Rodrigues(...)` function.
  * The information on this `wikipedia rotation formalisms <https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rodrigues_vector>`_ page and this `wikipedia Rodrigues' rotation formula <https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula>`_ are a starting point for reading more about the Rodrigues convention for representing rotations.


Robot pose estimation
*********************

Letting :math:`t_{\mathrm{vec}}` denote :code:`tvec` and :math:`R` denote :code:`Rmat` from above, then a vector expressed in the maker frame coordinates can be transformed to the camera frame coordinates as:

  .. math::

     \begin{bmatrix}x\\y\\z\end{bmatrix}^{\,\mathrm{camera}\,\mathrm{frame}} = t_{\mathrm{vec}} + R \, \begin{bmatrix}x\\y\\z\end{bmatrix}^{\,\mathrm{marker}\,\mathrm{frame}}

Based on a known location and rotation of the particular ArUco marker relative to the world frame, it is possible to compute an estimate of the camera's location within the world frame, and hence an estimate of robot's pose on which the camera is mounted.



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
