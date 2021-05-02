#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Template Python node to capture and publish video
#
# ----------------------------------------------------------------------------

# Import the ROS-Python package
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image

# Import opencv
import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge



# DEFINE THE PARAMETERS
# > For the number of the USB camera device
#   i.e., for /dev/video0, this parameter should be 0
USB_CAMERA_DEVICE_NUMBER = 0

# > For where to save images captured by the camera
#   Note: ensure that this path already exists
#   Note: images are only saved when a message is received
#         on the "request_save_image" topic.
SAVE_IMAGE_PATH = "/home/asc01/saved_camera_images/"

# > A flag for whether to display the images captured
SHOULD_SHOW_IMAGES = False



class TemplateCameraCapture:

    def __init__(self):
        
        # Initialise a publisher for the images
        self.image_publisher = rospy.Publisher("/global_namespace"+"/camera_image", Image, queue_size=10)

        # Initialise a subscriber for flagging when to save an image
        rospy.Subscriber("/global_namespace"+"/request_save_image", UInt32, self.requestSaveImageSubscriberCallback)
        # > For convenience, the command line can be used to trigger this subscriber
        #   by publishing a message to the "request_save_image" as follows:
        #
        # rostopic pub /global_namespace/request_save_image std_msgs/UInt32 "data: 1" 

        # Initialise varaibles for managing the saving of an image
        self.save_image_counter = 0
        self.should_save_image = False

        # Specify the details for camera to capture from
        
        # > For capturing from a USB camera:
        self.camera_setup = USB_CAMERA_DEVICE_NUMBER
        
        # > For capturing from the CSI camera port:
        #   > Gstreamer for capture video
        #   > sensor-id=0 for CAM0 and sensor-id=1 for CAM1
        #   > This is not optimized, but it does work.
        #self.camera_setup = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=12/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, width = 800, height=600, format =BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

        # Initialise video capture from the camera
        self.cam=cv2.VideoCapture(self.camera_setup)

        # Initlaise the OpenCV<->ROS bridge
        self.cv_bridge = CvBridge()

        # Display the status
        rospy.loginfo("[TEMPLATE CAMERA CAPTURE] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(3.0), self.timerCallbackForPublishing)



    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        # Read the camera frame
        rospy.loginfo('[TEMPLATE CAMERA CAPTURE] Now reading camera frame')
        return_flag , current_frame = self.cam.read()

        # Check if the camera frame was successfully read
        if (return_flag == True):
            # Publish the camera frame
            rospy.loginfo('[TEMPLATE CAMERA CAPTURE] Now publishing camera frame')
            self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame))

            # Check if the camera frame contains a calibration
            # chessboard that can be detected by OpenCV
            # > Convert the image to gray
            current_frame_as_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            # > Specify the flags for the find chessboard function
            find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH #+ cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
            # > Call the find the chess board corners
            #   > The second argument is the grid size of internl corner
            #     points that the function should search for
            ret, corners = cv2.findChessboardCorners(current_frame_as_gray, (6,5), flags=find_flags)
            # If found, then set the save image flag to true
            if ret == True:
                rospy.loginfo("[TEMPLATE CAMERA CAPTURE] Chessboard FOUND, this image will be saved")
                self.should_save_image = True
            #else:
            #    rospy.loginfo("[TEMPLATE CAMERA CAPTURE] Chessboard NOT found")

            # Save the camera frame if requested
            if (self.should_save_image):
                # Increment the image counter
                self.save_image_counter += 1
                # Write the image to file
                temp_filename = SAVE_IMAGE_PATH + "image" + str(self.save_image_counter) + ".jpg"
                cv2.imwrite(temp_filename,current_frame)
                # Display the path to where the image was saved
                rospy.loginfo("[TEMPLATE CAMERA CAPTURE] Save camera frame to: " + temp_filename)
                # Reset the flag to false
                self.should_save_image = False

            # Display the camera frame
            if (SHOULD_SHOW_IMAGES):
                rospy.loginfo("[TEMPATE CAMERA CAPTURE] Now displaying camera frame")
                cv2.imshow("CAM 0", current_frame)
        else:
            # Display an error message
            rospy.loginfo('[TEMPLATE CAMERA CAPTURE] ERROR occurred during self.cam.read()')



    # Respond to subscriber receiving a message
    def requestSaveImageSubscriberCallback(self, msg):
        rospy.loginfo("[TEMPLATE CAMERA CAPTURE] Request receieved to save the next image")
        # Set the flag for saving an image
        self.should_save_image = True



if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "template_camera_capture"
    rospy.init_node(node_name)
    template_camera_capture = TemplateCameraCapture()
    # Spin as a single-threaded node
    rospy.spin()
