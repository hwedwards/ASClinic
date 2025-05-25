#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PlantDetection
from asclinic_pkg.msg import ServoPulseWidth
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import glob

# Nested dictionary mapping plant IDs to location-specific lists of servo pulse widths
# Populate with: plant_id: { location: [pulse_width1, pulse_width2, ...], ... }
SERVO_PULSE_WIDTH_MAP = {
    1: { 1: [1950, 2000, 2150], 2: [750, 825, 950] },
}

BLURRINESS_THRESHOLD = 80  # Ignore frames with Laplacian variance below this threshold (decided from calculating the variance of the training set of model and seeing which is acceptable)
CAPTURE_COUNT = 5  # Number of good frames to collect before stopping

servo_pub = None
bridge = CvBridge()
save_dir_base = "/home/asc/saved_camera_images/saved_plant_images/raw_images"

class PlantCollector:
    def __init__(self):
        self.current_plant_id = 0
        self.current_location = 0
        self.positions = []
        self.index = 0
        self.raw_frames = []
        self.capture_count = CAPTURE_COUNT
        self.done_pub = rospy.Publisher("/asc/plant_done", Bool, queue_size=1)

    def start_collection(self, plant_id, location):
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
        self._next_position()

    def _next_position(self):
        if self.index >= len(self.positions):
            # all positions done: reset servo and signal done
            servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=1500))
            rospy.sleep(1.0)
            self.done_pub.publish(Bool(data=True))
            return
        # move to this servo position
        pulse = self.positions[self.index]
        servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=pulse))
        rospy.loginfo(f"Moving servo to {pulse} at plant {self.current_plant_id}, loc {self.current_location}, pos idx {self.index}")
        rospy.sleep(1.0)
        # clear buffer and start capture
        self.raw_frames = []
        self.collecting = True

    def process_frame(self, frame):
        if not getattr(self, 'collecting', False):
            return
        self.raw_frames.append(frame.copy())
        rospy.loginfo(f"Captured frame {len(self.raw_frames)}/{self.capture_count}")
        if len(self.raw_frames) >= self.capture_count:
            # stop collecting
            self.collecting = False
            # pick least blurry (max Laplacian variance)
            best_var = -1
            best_img = None
            for img in self.raw_frames:
                var = cv2.Laplacian(img, cv2.CV_64F).var()
                if var > best_var:
                    best_var = var
                    best_img = img
            # save best_img
            folder = os.path.join(save_dir_base, f"plant_{self.current_plant_id}_location_{self.current_location}")
            os.makedirs(folder, exist_ok=True)
            fname = f"plant_{self.current_plant_id}_location_{self.current_location}_position_{self.index}.jpg"
            cv2.imwrite(os.path.join(folder, fname), best_img)
            rospy.loginfo(f"Saved best (variance={best_var:.2f}) to {fname}")
            # advance to next position
            self.index += 1
            self._next_position()

plant_collector = PlantCollector()

def at_plant_callback(msg):
    if msg.plant_id == -1:
        return
    plant_collector.start_collection(msg.plant_id, msg.location)

def camera_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        return
    plant_collector.process_frame(frame)

def main():
    os.makedirs(save_dir_base, exist_ok=True)
    for f in glob.glob(os.path.join(save_dir_base, "*.jpg")):
        os.remove(f)

    rospy.init_node("plant_detector_node", anonymous=True)
    global servo_pub
    servo_pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=1)
    rospy.Subscriber("/asc/at_plant", PlantDetection, at_plant_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/asc/camera_image", Image, camera_callback, queue_size=1, buff_size=2**24)
    rospy.loginfo("Subscribed to /asc/at_plant and /asc/camera_image")

    rospy.spin()

if __name__ == '__main__':
    main()