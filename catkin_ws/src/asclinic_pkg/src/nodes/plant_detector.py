#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PlantDetection
from asclinic_pkg.msg import ServoPulseWidth
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from datetime import datetime
import glob
import threading
from std_msgs.msg import Bool


# Nested dictionary mapping plant IDs to location-specific lists of servo pulse widths
# Populate with: plant_id: { location: [pulse_width1, pulse_width2, ...], ... }
SERVO_PULSE_WIDTH_MAP = {
    1: { 0: [1950, 2000, 2150, 1500], 1: [750, 825, 950, 1500] },
}

# Will hold the servo command publisher
servo_pub = None

plant_id = 0
bridge = CvBridge()
model = YOLO("/home/asc/ASClinic/catkin_ws/src/asclinic_pkg/src/plant_detector/weights_2.pt")
save_dir_base = "/home/asc/saved_camera_images/saved_plant_images"

BLURRINESS_THRESHOLD = 80  # Ignore frames with Laplacian variance below this threshold (decided from calculating the variance of the training set of model and seeing which is acceptable)
CONFIDENCE_THRESHOLD = 0.7  # Discard detections with confidence below this threshold
BUG_CONFIDENCE_THRESHOLD = 0.4  # Lower confidence threshold for bugs (change this when model improves)
AREA_THRESHOLD = 100000  # Minimum area for a valid detection
FB_AREA_THRESHOLD = 10000  # Minimum area for flower/bug detection
NUM_PHOTOS_TO_TAKE = 3  # Number of good frames to collect before stopping


class PlantCollector:
    def __init__(self):
        self.lock = threading.RLock()
        self.current_plant_id = 0
        self.current_location = 0
        self.frames_info = []  # list of dicts: {'frame': ..., 'max_area': ..., 'conf': ..., 'detections': ..., 'location': int}
        self.good_frames_needed = NUM_PHOTOS_TO_TAKE
        self.area_threshold = AREA_THRESHOLD  
        self.characteristics = []  # to accumulate detection summaries across angles
        self.collecting = False
        self.collected = 0
        self.done_pub = rospy.Publisher("/asc/plant_done", Bool, queue_size=1)
        self.plant_location_results = {}  # maps plant_id to list of best-frame detections

    def reset(self):
        with self.lock:
            self.frames_info = []
            self.characteristics = []
            self.collected = 0

    def process_frame(self, frame):
        if not self.collecting:
            return
        rospy.loginfo("[YOLO Stream] Received image frame for processing")
        # Laplacian blur check for blurriness
        laplacian_var = cv2.Laplacian(frame, cv2.CV_64F).var()
        if laplacian_var < BLURRINESS_THRESHOLD:  # threshold for blurriness, adjust as needed
            rospy.loginfo(f"[YOLO Stream] Frame rejected due to blur: Laplacian variance = {laplacian_var:.2f}")
            return
        rospy.loginfo(f"[YOLO Stream] Laplacian variance = {laplacian_var:.2f} (passed threshold)")
        results = model(frame)[0]
        detections_summary = {'plant_type': None, 'bug_present': False, 'flower_present': False, 'confidences': []}
        has_good_detection = False
        areas = []
        plant_candidates = []
        for box in results.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            class_name = model.names[cls]
            is_bug = 'bug' in class_name.lower()
            # use lower confidence threshold for bugs only
            conf_thresh = BUG_CONFIDENCE_THRESHOLD if is_bug else CONFIDENCE_THRESHOLD
            is_plant = 'plant' in class_name.lower()
            is_bug_or_flower = 'bug' in class_name.lower() or 'flower' in class_name.lower()
            area_thresh = AREA_THRESHOLD if is_plant else FB_AREA_THRESHOLD
            if area > area_thresh and conf > conf_thresh: 
                has_good_detection = True
                areas.append(area)
                rospy.loginfo(f"[YOLO Stream] Accepted detection: class='{class_name}', conf={conf:.2f}, area={area}")
                detections_summary['confidences'].append(conf)
                if is_plant:
                    plant_candidates.append((class_name, conf))
                if 'bug' in class_name.lower():
                    detections_summary['bug_present'] = True
                    rospy.loginfo(f"[YOLO Stream] Bug detected: class_name='{class_name}', conf={conf:.2f}, area ={area}")
                if 'flower' in class_name.lower():
                    detections_summary['flower_present'] = True
                    rospy.loginfo(f"[YOLO Stream] Flower detected: class_name='{class_name}', conf={conf:.2f}, area ={area}")
        if plant_candidates:
            best_plant = max(plant_candidates, key=lambda x: x[1])
            detections_summary['plant_type'] = best_plant[0]
            rospy.loginfo(f"[YOLO Stream] Selected plant type: '{best_plant[0]}' with confidence {best_plant[1]:.2f}")
        if has_good_detection:
            max_area = max(areas, default=0)
            max_conf = max(detections_summary['confidences'], default=0)
            with self.lock:
                location = self.current_location
                self.frames_info.append({'frame': frame.copy(), 'max_area': max_area, 'conf': max_conf, 'detections': detections_summary, 'location': location})
                self.characteristics.append(detections_summary)
                self.collected += 1
                rospy.loginfo(f"[YOLO Stream] Good frame collected ({self.collected}/{self.good_frames_needed})")
                if self.collected >= self.good_frames_needed:
                    self.collecting = False
                    self.save_best_frame_and_summary()

    def save_best_frame_and_summary(self):
        rospy.loginfo("[YOLO Stream] Entering save_best_frame_and_summary()")
        with self.lock:
            if not self.frames_info:
                rospy.logwarn("[YOLO Stream] No good frames collected to save")
                return
            best_frame_info = max(self.frames_info, key=lambda x: cv2.Laplacian(x['frame'], cv2.CV_64F).var())
            rospy.loginfo(f"[YOLO Stream] Saving best frame from location {best_frame_info['location']}")
            plant_folder = os.path.join(save_dir_base, f"plant_{self.current_plant_id:03d}")
            os.makedirs(plant_folder, exist_ok=True)
            image_path = os.path.join(plant_folder, f"location_{best_frame_info['location']:02d}.jpg")
            results = model(best_frame_info['frame'])[0]
            annotated = results.plot()
            for box in results.boxes:
                # extract coords and compute area
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                # put the area as text just above the box
                cv2.putText(
                    annotated,
                    str(area),
                    (x1, y1 - 10),                   # position
                    cv2.FONT_HERSHEY_SIMPLEX,        # font
                    0.5,                             # font scale
                    (0, 255, 0),                     # color (green)
                    1                                # thickness
                )
            success = cv2.imwrite(image_path, annotated)
            if success:
                rospy.loginfo(f"[YOLO Stream] Image successfully saved to: {image_path}")
            else:
                rospy.logerr(f"[YOLO Stream] Failed to save image to: {image_path}")

            location = best_frame_info['location']
            flower_present = any(f['flower_present'] for f in self.characteristics)
            bug_present = any(f['bug_present'] for f in self.characteristics)
            plant_type = best_frame_info['detections']['plant_type']
            avg_conf = sum(best_frame_info['detections']['confidences']) / len(best_frame_info['detections']['confidences']) if best_frame_info['detections']['confidences'] else 0.0

            location_csv_path = os.path.join(plant_folder, "locations.csv")
            write_header = not os.path.exists(location_csv_path)
            with open(location_csv_path, "a") as f:
                if write_header:
                    f.write("location,plant_type,bug_present,flower_present,avg_confidence\n")
                f.write(f"{location},{plant_type},{bug_present},{flower_present},{avg_conf:.2f}\n")

            self.done_pub.publish(Bool(data=True))

plant_collector = PlantCollector()

def at_plant_callback(msg):
    rospy.loginfo(f"[YOLO Stream] Received /asc/at_plant message for plant_id {msg.plant_id}")
    plant_collector.done_pub.publish(Bool(data=False))
    rospy.loginfo(f"[YOLO Stream] Beginning frame collection for plant_id {msg.plant_id}")
    plant_collector.current_plant_id = msg.plant_id
    plant_collector.current_location = msg.location
    plant_collector.reset()
    plant_collector.collecting = True

    # Publish all configured pulse widths for this plant_id and location
    servo_list = SERVO_PULSE_WIDTH_MAP.get(msg.plant_id, {}).get(msg.location)
    if servo_list:
        for pulse_width in servo_list:
            servo_msg = ServoPulseWidth(channel=3, pulse_width_in_microseconds=pulse_width)
            servo_pub.publish(servo_msg)
            rospy.loginfo(f"[YOLO Stream] Published servo pulse width for plant_id {msg.plant_id}, location {msg.location}: {pulse_width} μs")
            rospy.sleep(2)
        plant_collector.done_pub.publish(Bool(data=True))
    else:
        rospy.logwarn(f"[YOLO Stream] No servo pulse widths configured for plant_id {msg.plant_id}, location {msg.location}")

    # Wait until collection is done
    while plant_collector.collecting and not rospy.is_shutdown():
        rospy.sleep(0.1)
    plant_collector.collecting = False
    rospy.loginfo(f"[YOLO Stream] Completed frame collection for plant_id {msg.plant_id}")

def camera_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logwarn(f"[YOLO Stream] Failed to convert image: {e}")
        return
    plant_collector.process_frame(frame)

def main():
    os.makedirs(save_dir_base, exist_ok=True)
    for f in glob.glob(os.path.join(save_dir_base, "*.jpg")):
        os.remove(f)

    rospy.init_node("yolo_stream_node", anonymous=True)
    # Initialize servo publisher
    global servo_pub
    servo_pub = rospy.Publisher("/asc/set_servo_pulse_width", ServoPulseWidth, queue_size=1)
    rospy.Subscriber("/asc/at_plant", PlantDetection, at_plant_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/asc/camera_image", Image, camera_callback, queue_size=1, buff_size=2**24)
    rospy.loginfo("[YOLO Stream] Subscribed to /asc/at_plant and /asc/camera_image")

    rospy.spin()

if __name__ == '__main__':
    main()