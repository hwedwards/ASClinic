#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PlantDetection, ServoPulseWidth
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from datetime import datetime
import cv2
import os
import threading

# map of plant → location → list of pulse widths
SERVO_PULSE_WIDTH_MAP = {
    1: { 1: [1950, 2000, 2150, 1500], 2: [750, 825, 950, 1500] },
}

# where to dump your images
save_dir_base = "/home/asc/saved_camera_images/saved_plant_images"

# for converting ROS images to OpenCV
bridge = CvBridge()

# will get set in main()
servo_pub = None
plant_collector = None


class PlantCollector:
    def __init__(self):
        self.lock = threading.Lock()
        self.collecting = False
        self.images = []

        self.plant_id = None
        self.location = None
        self.positions = []
        self.index = 0

        # notify when we're done
        self.plant_done_pub = rospy.Publisher("/asc/plant_done", Bool, queue_size=1)

    def at_plant_callback(self, msg: PlantDetection):
        if not msg.are_at_plant:
            return

        rospy.loginfo(f"[YOLO Stream] Starting capture for plant_id={msg.plant_id}, location={msg.location}")
        self.plant_id = msg.plant_id
        self.location = msg.location
        self.positions = SERVO_PULSE_WIDTH_MAP.get(self.plant_id, {}).get(self.location, [])

        if not self.positions:
            rospy.logwarn(f"[YOLO Stream] No servo positions for plant={self.plant_id}, loc={self.location}")
            # immediately signal done so upstream can keep going
            self.plant_done_pub.publish(Bool(data=True))
            return

        self.index = 0
        self._next_position()

    def _next_position(self):
        if self.index >= len(self.positions):
            rospy.loginfo(f"[YOLO Stream] Finished all positions for plant={self.plant_id}, loc={self.location}")
            self.collecting = False
            # signal completion
            self.plant_done_pub.publish(Bool(data=True))
            rospy.loginfo("[YOLO Stream] Published plant_done=True")
            return

        pulse = self.positions[self.index]
        spw = ServoPulseWidth(channel = 3, pulse_width_in_microseconds=pulse)
        servo_pub.publish(spw)
        rospy.loginfo(f"[YOLO Stream] Moved servo → {pulse} μs (idx {self.index+1}/{len(self.positions)})")
        rospy.sleep(1.0)  # allow servo to settle

        with self.lock:
            self.images = []
        self.collecting = True

    def process_frame(self, frame):
        if not self.collecting:
            return

        with self.lock:
            self.images.append(frame.copy())
            rospy.loginfo(f"[YOLO Stream] Collected image {len(self.images)}/3 at pulse {self.positions[self.index]}")

            if len(self.images) >= 3:
                # save all 3
                for i, img in enumerate(self.images):
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    fname = f"plant{self.plant_id}_loc{self.location}_pulse{self.positions[self.index]}_{ts}_{i}.jpg"
                    cv2.imwrite(os.path.join(save_dir_base, fname), img)
                rospy.loginfo(f"[YOLO Stream] Saved 3 images for pulse {self.positions[self.index]}")

                # move on
                self.collecting = False
                self.index += 1
                rospy.sleep(0.5)
                self._next_position()


def camera_callback(msg: Image):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logwarn(f"[YOLO Stream] Image convert failed: {e}")
        return
    plant_collector.process_frame(frame)


def main():
    global servo_pub, plant_collector

    rospy.init_node("yolo_stream_node", anonymous=True)

    # ensure clean output directory
    os.makedirs(save_dir_base, exist_ok=True)
    for f in os.listdir(save_dir_base):
        if f.lower().endswith((".jpg", ".png")):
            os.remove(os.path.join(save_dir_base, f))

    servo_pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=1)
    plant_collector = PlantCollector()

    rospy.Subscriber("/asc/at_plant", PlantDetection, plant_collector.at_plant_callback,
                     queue_size=1, buff_size=2**24)
    rospy.Subscriber("/asc/camera_image", Image, camera_callback,
                     queue_size=1, buff_size=2**24)

    rospy.loginfo("[YOLO Stream] Ready. Waiting for /asc/at_plant …")
    rospy.spin()


if __name__ == "__main__":
    main()