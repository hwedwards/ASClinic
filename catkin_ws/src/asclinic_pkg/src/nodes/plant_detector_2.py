#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PlantDetection
from asclinic_pkg.msg import ServoPulseWidth
from std_msgs.msg import Bool
from std_msgs.msg import UInt32
import os
import glob
import cv2

# Nested dictionary mapping plant IDs to location-specific lists of servo pulse widths
SERVO_PULSE_WIDTH_MAP = {
    1: { 1: [1800, 2000], 2: [850, 1050] },
}

BLURRINESS_THRESHOLD = 80  # Ignore frames with Laplacian variance below this threshold (decided from calculating the variance of the training set of model and seeing which is acceptable)
CAPTURE_COUNT = 5  # Number of good frames to collect before stopping

servo_pub = None
save_dir_base = "/home/asc/saved_camera_images/saved_plant_images/raw_images"

class PlantCollector:
    def __init__(self):
        self.collecting = False
        self.current_plant_id = 1
        self.current_location = 1
        self.positions = []
        self.index = 0
        self.capture_count = CAPTURE_COUNT
        self.done_pub = rospy.Publisher("/asc/plant_done", Bool, queue_size=1)
        # publisher to request camera_capture to save an image
        self.save_req_pub = rospy.Publisher("/asc/request_save_image", UInt32, queue_size=1)
        self.last_command_time = rospy.Time(0)
        self.plant_done_timer = None

    def _publish_not_done(self, event):
        # continuously signal not done while sweeping
        self.done_pub.publish(Bool(data=False))

    def start_collection(self, plant_id, location):
        self.done_pub.publish(Bool(data=False))  # set done to false - suspend aruco positioning
        self.current_plant_id = plant_id
        self.current_location = location
        # prepare positions
        self.positions = SERVO_PULSE_WIDTH_MAP.get(plant_id, {}).get(location, [])
        self.index = 0
        if not self.positions:
            # reset servo and signal done
            servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=1500))
            rospy.sleep(1.0)
            self.done_pub.publish(Bool(data=True))
            return
        rospy.loginfo(f"[YOLO Stream] Positions list: {self.positions}")
        self._next_position()

    def _next_position(self):
        if self.index >= len(self.positions):
            # disable capturing before home reset
            self.collecting = False
            # all positions done: reset servo and signal done
            servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=1500))
            rospy.sleep(1.0)
            if self.plant_done_timer:
                self.plant_done_timer.shutdown()
                self.plant_done_timer = None
            self.done_pub.publish(Bool(data=True))
            return
        # move to this servo position
        pulse = self.positions[self.index]
        rospy.loginfo(f"[YOLO Stream] Position {self.index+1}/{len(self.positions)}: moving servo to {pulse}")
        servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=pulse))
        rospy.sleep(3.0)  # wait for servo to settle

        # start continuously publishing False while not at home
        if pulse != 1500:
            if self.plant_done_timer:
                self.plant_done_timer.shutdown()
            self.plant_done_timer = rospy.Timer(rospy.Duration(1.0),
                                                self._publish_not_done)

        # request saving of CAPTURE_COUNT images
        for i in range(self.capture_count):
            self.save_req_pub.publish(UInt32(data=1))
            rospy.loginfo(f"[YOLO Stream] Requested save image {i+1}/{self.capture_count}")
            rospy.sleep(1.0)

        # filter and save the least blurry of the batch
        self._filter_and_save_best()

        # advance to next position
        self.index += 1
        self._next_position()

    def _filter_and_save_best(self):
        # find raw images
        pattern = os.path.join(save_dir_base, "*.jpg")
        files = sorted(glob.glob(pattern), key=os.path.getmtime)
        if not files:
            rospy.logwarn("[YOLO Stream] No images found to filter")
            return
        # select last CAPTURE_COUNT images, or all if fewer
        batch = files[-self.capture_count:] if len(files) >= self.capture_count else files[:]
        best_file = None
        best_var = -1.0
        for fpath in batch:
            img = cv2.imread(fpath)
            if img is None:
                continue
            var = cv2.Laplacian(img, cv2.CV_64F).var()
            if var > best_var:
                best_var = var
                best_file = fpath
        if not best_file:
            rospy.logwarn("[YOLO Stream] No valid images to filter")
            return

        # prepare output directory
        filtered_dir = "/home/asc/saved_camera_images/saved_plant_images/filtered_images"
        os.makedirs(filtered_dir, exist_ok=True)

        # construct output filename
        out_name = f"plant_{self.current_plant_id}_location_{self.current_location}_position_{self.index}.jpg"
        out_path = os.path.join(filtered_dir, out_name)

        # save the best image
        img = cv2.imread(best_file)
        cv2.imwrite(out_path, img)
        rospy.loginfo(f"[YOLO Stream] Saved filtered best image ({best_var:.2f}) to {out_path}")

plant_collector = PlantCollector()

def at_plant_callback(msg):
    if msg.plant_id == -1:
        return
    plant_collector.start_collection(msg.plant_id, msg.location)

def main():
    # Clear raw_images directory
    if os.path.exists(save_dir_base):
        for f in glob.glob(os.path.join(save_dir_base, "*")):
            os.remove(f)
    else:
        os.makedirs(save_dir_base, exist_ok=True)

    # Clear filtered_images directory
    filtered_dir = "/home/asc/saved_camera_images/saved_plant_images/filtered_images"
    if os.path.exists(filtered_dir):
        for f in glob.glob(os.path.join(filtered_dir, "*")):
            os.remove(f)
    else:
        os.makedirs(filtered_dir, exist_ok=True)

    rospy.init_node("plant_detector_node", anonymous=True)
    global servo_pub
    servo_pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=1)
    rospy.Subscriber("/asc/at_plant", PlantDetection, at_plant_callback, queue_size=1, buff_size=1)
    rospy.loginfo("Subscribed to /asc/at_plant")

    rospy.spin()

if __name__ == '__main__':
    main()