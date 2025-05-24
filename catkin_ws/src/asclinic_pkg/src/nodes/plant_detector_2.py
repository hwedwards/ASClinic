#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PlantDetection, ServoPulseWidth
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import os
from datetime import datetime

# map of plant → location → list of pulse widths
SERVO_PULSE_WIDTH_MAP = {
    1: {1: [1950, 2000, 2150, 1500], 2: [750, 825, 950, 1500]},
}

# how many images to take per position
NUM_IMAGES = 3

# output directory
save_dir_base = "/home/asc/saved_camera_images/saved_plant_images"

bridge = CvBridge()
servo_pub = None
images_taken = 0
current_plant = None
current_location = None

def at_plant_callback(msg: PlantDetection):
    global images_taken, current_plant, current_location
    rospy.loginfo(f"[Image Capture] Starting capture for plant {msg.plant_id}, location {msg.location}")
    servo_list = SERVO_PULSE_WIDTH_MAP.get(msg.plant_id, {}).get(msg.location, [])
    if not servo_list:
        rospy.logwarn(f"[Image Capture] No servo positions for plant {msg.plant_id}, location {msg.location}")
        rospy.Publisher("/asc/plant_done", Bool, queue_size=1).publish(Bool(data=True))
        return

    current_plant = msg.plant_id
    current_location = msg.location

    # ensure output dir exists
    plant_folder = os.path.join(save_dir_base, f"plant_{msg.plant_id}_{msg.location}")
    os.makedirs(plant_folder, exist_ok=True)
    # clear old images
    for f in os.listdir(plant_folder):
        if f.endswith(".jpg"):
            os.remove(os.path.join(plant_folder, f))

    # capture loop
    for pulse in servo_list:
        rospy.loginfo(f"[Image Capture] Moving servo to {pulse}µs")
        servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=pulse))
        rospy.sleep(1.0)

        images_taken = 0
        start = rospy.Time.now()
        while images_taken < NUM_IMAGES and (rospy.Time.now() - start).to_sec() < 10.0:
            rospy.sleep(0.1)

    rospy.loginfo(f"[Image Capture] Finished all positions")
    rospy.Publisher("/asc/plant_done", Bool, queue_size=1).publish(Bool(data=True))

def camera_callback(msg: Image):
    global images_taken
    if images_taken >= NUM_IMAGES:
        return
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logwarn(f"[Image Capture] Failed to convert image: {e}")
        return

    # save the frame
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    fname = f"img_{ts}_{images_taken}.jpg"
    plant_folder = os.path.join(save_dir_base, f"plant_{current_plant}_{current_location}")
    path = os.path.join(plant_folder, fname)
    if cv2.imwrite(path, frame):
        rospy.loginfo(f"[Image Capture] Saved image {images_taken+1}/{NUM_IMAGES}: {path}")
        images_taken += 1
    else:
        rospy.logwarn(f"[Image Capture] Failed to write {path}")

def main():
    global servo_pub, current_plant, current_location
    current_plant = None
    current_location = None
    rospy.init_node("image_capture_node")
    servo_pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=1)
    rospy.Subscriber("/asc/at_plant", PlantDetection, at_plant_callback)
    rospy.Subscriber("/asc/camera_image", Image, camera_callback)
    rospy.loginfo("[Image Capture] Node ready, waiting for /asc/at_plant")
    rospy.spin()

if __name__ == "__main__":
    main()