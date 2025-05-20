#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from asclinic_pkg.msg import PlantDetection
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from datetime import datetime
import glob
import threading

plant_id = 0
bridge = CvBridge()
model = YOLO("/home/asc/ASClinic/catkin_ws/src/asclinic_pkg/src/plant_detector/weights.pt")
save_dir_base = "/home/asc/saved_camera_images/saved_plant_images"

BLURRINESS_THRESHOLD = 20  # Ignore frames with Laplacian variance below this threshold
CONFIDENCE_THRESHOLD = 0.5  # Discard detections with confidence below this threshold
AREA_THRESHOLD = 5000  # Minimum area for a valid detection
LAPLACIAN_WEIGHT = 10  # Weight for combining area and sharpness in best frame selection


class PlantCollector:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_plant_id = 0
        self.frames_info = []  # list of dicts: {'frame': ..., 'max_area': ..., 'conf': ..., 'detections': ..., 'location': int}
        self.good_frames_needed = 10
        self.area_threshold = AREA_THRESHOLD  
        self.characteristics = []  # to accumulate detection summaries across angles
        self.collecting = False
        self.collected = 0

    def reset(self):
        with self.lock:
            self.frames_info = []
            self.characteristics = []
            self.collected = 0

    def process_frame(self, frame):
        if not self.collecting:
            return
        # Laplacian blur check for blurriness
        laplacian_var = cv2.Laplacian(frame, cv2.CV_64F).var()
        if laplacian_var < BLURRINESS_THRESHOLD:  # threshold for blurriness, adjust as needed
            return
        results = model(frame)[0]
        detections_summary = {'plant_type': None, 'bug_present': False, 'flower_present': False, 'confidences': []}
        has_good_detection = False
        areas = []
        for box in results.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            if area > self.area_threshold and conf > CONFIDENCE_THRESHOLD: 
                has_good_detection = True
                areas.append(area)
                class_name = model.names[cls]
                detections_summary['confidences'].append(conf)
                if 'plant' in class_name.lower():
                    detections_summary['plant_type'] = class_name
                if 'bug' in class_name.lower():
                    detections_summary['bug_present'] = True
                if 'flower' in class_name.lower():
                    detections_summary['flower_present'] = True
        if has_good_detection:
            max_area = max(areas, default=0)
            max_conf = max(detections_summary['confidences'], default=0)
            with self.lock:
                location = self.collected + 1  # now an int
                self.frames_info.append({'frame': frame.copy(), 'max_area': max_area, 'conf': max_conf, 'detections': detections_summary, 'location': location})
                self.characteristics.append(detections_summary)
                self.collected += 1
                if self.collected >= self.good_frames_needed:
                    self.collecting = False
                    self.save_best_frame_and_summary()

    def save_best_frame_and_summary(self):
        with self.lock:
            if not self.frames_info:
                rospy.logwarn("[YOLO Stream] No good frames collected to save")
                return
            best_frame_info = max(self.frames_info, key=lambda x: x['max_area'] + LAPLACIAN_WEIGHT * cv2.Laplacian(x['frame'], cv2.CV_64F).var())
            plant_folder = os.path.join(save_dir_base, f"plant_{self.current_plant_id:03d}")
            os.makedirs(plant_folder, exist_ok=True)
            image_path = os.path.join(plant_folder, f"location_{best_frame_info['location']:02d}.jpg")
            cv2.imwrite(image_path, best_frame_info['frame'])
            # Aggregate characteristics across all frames
            plant_types = set()
            bug_present = False
            flower_present = False
            all_confidences = []
            for char in self.characteristics:
                if char['plant_type']:
                    plant_types.add(char['plant_type'])
                if char['bug_present']:
                    bug_present = True
                if char['flower_present']:
                    flower_present = True
                all_confidences.extend(char['confidences'])
            avg_confidence = sum(all_confidences)/len(all_confidences) if all_confidences else 0.0
            csv_path = os.path.join(plant_folder, "characteristics.csv")
            with open(csv_path, "a") as f:
                plant_types_str = ";".join(sorted(plant_types)) if plant_types else "Unknown"
                f.write(f"{plant_types_str},{bug_present},{flower_present},{avg_confidence:.2f}\n")

plant_collector = PlantCollector()

def at_plant_callback(msg):
    global plant_id
    rospy.loginfo(f"[YOLO Stream] Received /asc/at_plant message for plant_id {plant_id}")
    plant_collector.current_plant_id = plant_id
    plant_collector.reset()
    plant_collector.collecting = True
    # Wait until collection is done
    while plant_collector.collecting and not rospy.is_shutdown():
        rospy.sleep(0.1)
    plant_collector.collecting = False
    plant_id += 1

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
    rospy.Subscriber("/asc/at_plant", PlantDetection, at_plant_callback, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/asc/camera_image", Image, camera_callback, queue_size=1, buff_size=2**24)
    rospy.loginfo("[YOLO Stream] Subscribed to /asc/at_plant and /asc/camera_image")

    rospy.spin()

if __name__ == '__main__':
    main()