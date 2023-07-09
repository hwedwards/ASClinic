.. _building-block-camera-calibration:

Camera calibration
==================

As the computer onboard is the robot is generally **not** connected to an external screen, it may at first seem difficult and unintuitive to work with data from the camera. You can consider setting up a remote desktop connection to the robot, and this will make certain parts of the workflow more like using you personal computer. But at the end of the day, the majority of your interface to the robot is through terminal commands and code, for which opening multiple :code:`ssh` connections is sufficient.


Install OpenCV
**************

Mostly likely this is already installed because it is included in both the :ref:`software-installation-manual` and :ref:`software-installation-script`. If not already installed, follow the :ref:`install_opencv_python` instructions.


Capture camera images
*********************

In order to calibrate the camera, you first need to be able to capture images from the camera. Before proceeding, follow the :ref:`building-block-usb-camera-settings` page to familiarise with setting available on the camera you are using, for example, whether or not it has auto-focus.

To capture images from the camera and save those images to file, the template python3 ROS node :code:`camera_capture.py` is provided. This file is located in the repository at the relative path:

.. code-block:: bash

  catkin_ws/src/asclinic_pkg/src/nodes/


The file is extensively commented and the comments serve as the documentation for how to edit the file to implement your use case. **Before running the camera capture node,** you should note the following parts of the code, adjusting as required:

* Adjust the parameters at the top of the file to match your use case. Namely:

  * Check that the :code:`USB_CAMERA_DEVICE_NUMBER` agrees with that of the camera plugged into your robot.

  * Adjust the values of :code:`DESIRED_CAMERA_FRAME_HEIGHT` and :code:`DESIRED_CAMERA_FRAME_WIDTH` to be the resolution that you desire.

  * Adjust the value of :code:`DESIRED_CAMERA_FRAME_FPS` to be the frame rate that you desire. **Note that** most cameras do not allow any arbitrary value for the frame per second (fps) property, and generally operate at the fps closest to your request, for example, 5 or 30 fps.

  * Adjust the path :code:`SAVE_IMAGE_PATH` to be appropriate for your robot and ensure that the path exists.

* The subscriber initialised within the :code:`__init__` function for the topic :code:`request_save_image` provides a convenient way to save images to file. If you publish a message to this topic, then the node will save the next camera image to file. You do **NOT** need to create a separate node for publishing a message to this topic. Instead, you can publish to this topic by entering the following in a separate terminal window that is logged in to the robot:

.. code-block::

  rostopic pub /global_namespace/request_save_image std_msgs/UInt32 "data: 1"

* The camera images are captured (and saved) within the :code:`timerCallbackForPublishing` function. Hence you can adjust the frequency of the camera captures by adjusting the :code:`rospy.Duration` of the timer where it is initialised within the :code:`__init__` function.

* The lines of code for saving images is within the :code:`if (self.should_save_image):` statement. There you see that the file name is index by a number that increments with each imaged saved to file. Hence, within one launch of the node, files will not be over-written. **However**, when you launch the node another time, it will start from index 1 again and hence over-write any previously saved imaged.


The "camera capture" node is launched by the file :code:`camera_capture.launch`, and is hence launched by the command:

.. code-block::

  roslaunch asclinic_pkg camera_capture.launch


Capture chessboard images
*************************

The first step for calibrating the intrinsic parameters of a camera is to collect multiple images (usually 10-20 images is suggested) of a chessboard from various angles. These images can then be post-processed to locate the internal corners of the chessboard within the image.

OpenCV includes a function that finds the internal chessboard corners, namely :code:`cv2.findChessboardCorners(...)`, however, this function can be quite sensitive to lighting conditions of the captured chessboard image. Hence the :code:`camera_capture.py` ROS node includes a section that saves any image of a chessboard that is successfully processed by the function :code:`cv2.findChessboardCorners(...)`. To enable this section of the code, simply adjust the parameter :code:`SHOULD_SAVE_CHESSBOARD_IMAGES` at the top of the file to be :code:`True`. The following code snippet from :code:`camera_capture.py` shows the steps implemented for processing an image, i.e., the variable :code:`current_frame`, to locate chessboard corners.

.. code-block:: python

  if (SHOULD_SAVE_CHESSBOARD_IMAGES):
    # > Convert the image to gray scale
    current_frame_as_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    # > Specify the flags for the find chessboard function
    find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    # > Call the find the chessboard corners function
    #   > The second argument is the grid size of internal corner
    #     points that the function should search for
    ret, corners = cv2.findChessboardCorners(current_frame_as_gray, (6,5), flags=find_flags)
    # > If found, then set the save image flag to true
    if (ret == True):
       rospy.loginfo("[CAMERA CAPTURE] Chessboard FOUND, this image will be saved")
       self.should_save_image = True

The there two important options that can be adjusted within this code snippet:

* :code:`find_flags` variable: this specifies options for how the :code:`cv2.findChessboardCorners(...)` function processes the image. It is not clear that activating all three options indicated in the code snippet provides the most reliable detection of a chessboard. You can test with fewer options, or remove all the options using :code:`find_flags = None`

* :code:`(6,5)`, which is the second argument of the :code:`cv2.findChessboardCorners(...)` function: this specifies the size of the grid of internal chessboard corners that the function should search for. You should adjust this argument to be appropriate for the chessboard you are using. For example, a chessboard with 10x7 squares will have 9x6 internal corners. It can be beneficial to search for fewer than all the internal corners of the chessboard to allow for images that cut off part of the chessboard, and to increase the likelihood that the find chessboard corners function is successful.


Compute the intrinsic parameters
********************************

The calibration of a camera computes its intrinsic parameters based on an "offline" processing of static images that were captured by the camera. Hence you can perform the calibration on any machine and platform that you find most convenient.

To get the saved images from you robot onto your personal computer, you can use the secure copy command (:code:`scp`) as follows:

.. code-block::

  scp <username>@<ip_address>:<path_to_images> .

Where:

* :code:`<username>` is the username for your robot, for example :code:`asc01`

* :code:`<ip_address>` is the IP address of your robot, for example :code:`10.41.146.223`

* :code:`<path_to_images>` is the folder path and file name where the image data is saved. If this starts with a :code:`/`, then it is an absolute path on the robot. Otherwise, this is a path relative to :code:`/home/<username>`. For example, all images saved by the "camera capture" node can be copied by using the path :code:`saved_camera_images/image*`

* :code:`.` means that the copied files will be saved on your local machine in the current directory

For this example, the command is as follows:

.. code-block::

  scp asc01@10.41.146.223:saved_camera_images/image* .


Now that you have the images available on your local machine, you can process them and compute the camera's intrinsic parameters using a free, open-source, toolbox. Two popular options are:

* `OpenCV Camera Calibration Tutorial for python <https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html>`_

* `Camera Calibration Toolbox for Matlab <http://www.vision.caltech.edu/bouguetj/calib_doc/>`_


You should read the information provided at the first link above because it provides a good level of detail to explain and contextualize the camera calibration procedure. The code snippet below is taken directly from that link, and then "spruced up" with a few additional comments to explain the steps. You can use the code snippet below by following these steps on any computer with OpenCV installed (including on the robot's computer):

1. Place all the chessboard images into a separate folder.
2. Within that folder, create a python script with the contents of the code snippet below.
3. Adjust the parameter values for the number of chessboard rows and columns, (i.e., :code:`NUM_CB_ROWS` and :code:`NUM_CB_COLS`), and for the side length dimension of a single chessboard square (i.e., :code:`CB_SQUARE_SIDE_LENGTH`).
4. Execute the python script (depending on your machine, your install of OpenCV, and your default python version, you may need to run the script with :code:`python3`).
5. Check the processed images to see that the :code:`cv2.findChessboardCorners(..)` function correctly identified the internal corners of the chessboard.
6. Save or copy the intrinsic camera parameter in whichever format you will use them in your subsequent scripts.


.. code-block:: python

  import numpy as np
  import cv2
  import glob

  NUM_CB_ROWS = 6
  NUM_CB_COLS = 5
  CB_SQUARE_SIDE_LENGTH = 0.0232

  # Specify the termination criteria
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  # Prepare the "world frame" coordinates of the chessboard corners,
  # referred to as the "object points",
  # i.e., coordinates of the form (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0),
  # noting that the third coordinate is always zero by convention
  objp = np.zeros((NUM_CB_ROWS*NUM_CB_COLS,3), np.float32)
  objp[:,:2] = np.mgrid[0:NUM_CB_ROWS,0:NUM_CB_COLS].T.reshape(-1,2)

  # Scale by the side length of a single the chessboard square
  objp = objp * CB_SQUARE_SIDE_LENGTH

  # Print out the object points for a visual check:
  print("World frame coordinates to be used for the chessboard corners:")
  print("objp =")
  print(str(objp))

  # Initialze arrays to store object points and image points from all the images.
  objpoints = [] # 3d point in real world space
  imgpoints = [] # 2d points in image plane.

  # Get all the jpg images in the current directory
  images = glob.glob('*.jpg')

  # Iterate over all the images found
  for fname in images:
      print("\nNow processing image: " + fname)
      # Read in the image
      img = cv2.imread(fname)
      # Convert the image to gray scale
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      # Find the chess board corners
      ret, corners = cv2.findChessboardCorners(gray, (NUM_CB_ROWS,NUM_CB_COLS), None)
      # If found
      if ret == True:
          print("> Chessboard corners FOUND")
          # > Add object points
          objpoints.append(objp)
          # > Refine the image points and then add them
          corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
          imgpoints.append(corners)
          # > Draw the located corners onto the image
          cv2.drawChessboardCorners(img, (NUM_CB_ROWS,NUM_CB_COLS), corners2, ret)
          # > Save the image to file for later checking
          temp_filename = fname + "_with_corners.jpg"
          cv2.imwrite(temp_filename,img)
          print("> Saved image to: " + temp_filename)
          # > Display the image for immediate checking
          #cv2.imshow('img', img)
          # > Wait breifly before moving on to the next iteration
          cv2.waitKey(500)
      else:
          print("> Chessboard corners NOT found")

  print("\nNow computing the camera calibration.")
  # Call the function to calibrate the camera, this returns
  # > mtx and dist, which are the intrinsic camera parameters
  # > rvecs and tvecs, which are the extrinsic parameters for each image
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

  # Display the intrinsic camera parameters
  print("> Camera calibration complete.")
  print("> The intrinsic camera parameter estimates are:")
  print("mtx  = ")
  print(str(mtx))
  print("dist = " + str(dist))

  cv2.destroyAllWindows()



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
