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
import csv


# Nested dictionary mapping plant IDs to location-specific lists of servo pulse widths
# Populate with: plant_id: { location: [pulse_width1, pulse_width2, ...], ... }
SERVO_PULSE_WIDTH_MAP = {
    1: { 1: [1950, 2000, 2150], 2: [750, 825, 950] },
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
NUM_PHOTOS_TO_TAKE = 5  # Number of good frames to collect before stopping

# TO DO: 
# - Save all images with plants in them, not just the best one. i.e remove best frame logic and expand to good frames
# - Save all characteristics of the plant in a CSV file
# - Ensure that saving of images and characteristics is logically organised
# - Ensure that aruco positioning is suspended while servo is not at 1500
# - Add in plant and servo positions
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
        self.index = 0
        self.timeout_timer = None
        self.raw_frames = []      # store raw frames per position
        self.capture_count = 5    # number of frames to grab before inference
        # collects best_info dict from each servo position
        self.position_results = []

    def reset(self):
        with self.lock:
            self.frames_info = []
            self.characteristics = []
            self.collected = 0

    def start_collection(self, plant_id, location):
        with self.lock:
            self.current_plant_id = plant_id
            self.current_location = location
            self.reset()
            self.index = 0
            # clear per-location results
            self.position_results = []
            # load servo positions and ensure 1500µs is last
            raw_positions = SERVO_PULSE_WIDTH_MAP.get(plant_id, {}).get(location, [])
            if 1500 in raw_positions:
                # move all 1500 entries to the end
                positions_no1500 = [p for p in raw_positions if p != 1500]
                raw_positions = positions_no1500 + [1500]
            self.positions = raw_positions
            self.collecting = True
            if not self.positions:
                rospy.logwarn(f"[YOLO Stream] No positions for plant {plant_id}, loc {location}")
                # reset to home position before done
                rospy.loginfo("[YOLO Stream] No positions: resetting servo to 1500 µs")
                servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=1500))
                rospy.sleep(1.0)
                self.done_pub.publish(Bool(data=True))
                return
            self._next_position()

    def process_frame(self, frame):
        if not self.collecting:
            return
        # Phase 1: capture raw frames
        if len(self.raw_frames) < self.capture_count:
            self.raw_frames.append(frame.copy())
            rospy.loginfo(f"[YOLO Stream] Captured raw frame {len(self.raw_frames)}/{self.capture_count}")
            # once enough frames captured, switch to inference phase
            if len(self.raw_frames) >= self.capture_count:
                self.collecting = False
                rospy.loginfo("[YOLO Stream] Raw capture done, running inference on batch")
                self._infer_and_select()
            return

    def _infer_and_select(self):
        # run inference & filtering on captured frames
        self.frames_info = []
        for frame in self.raw_frames:
            laplacian_var = cv2.Laplacian(frame, cv2.CV_64F).var()
            if laplacian_var < BLURRINESS_THRESHOLD:
                continue
            results = model(frame)[0]
            detections_summary = {'plant_type': None, 'bug_present': False, 'flower_present': False, 'confidences': []}
            areas, plant_candidates = [], []
            for box in results.boxes:
                cls = int(box.cls); conf = float(box.conf)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2-x1)*(y2-y1)
                name = model.names[cls]
                if 'plant' in name.lower() and area>AREA_THRESHOLD and conf>CONFIDENCE_THRESHOLD:
                    plant_candidates.append((name, conf))
                    areas.append(area)
                    detections_summary['plant_type'] = name
                    detections_summary['confidences'].append(conf)
            if not plant_candidates:
                continue
            best_conf = max(detections_summary['confidences'])
            detections_summary['plant_type'] = max(plant_candidates, key=lambda x: x[1])[0]
            info = {
                'frame': frame.copy(),
                'max_area': max(areas),
                'conf': best_conf,
                'detections': detections_summary,
                'location': self.current_location
            }
            self.frames_info.append(info)
        # after batch inference, pick best frame this position
        if self.frames_info:
            best_info = max(self.frames_info, key=lambda x: x['max_area'])
            self.position_results.append(best_info)
        else:
            rospy.logwarn("[YOLO Stream] No valid plant detections at this angle")

        # advance to next position
        if self.timeout_timer:
            self.timeout_timer.shutdown()
        self.index += 1
        self._next_position()

    def save_location_summary(self):
        rospy.loginfo("[YOLO Stream] Saving best frame for plant/location")
        if not self.position_results:
            rospy.logwarn("[YOLO Stream] No detections for this location")
            return
        # pick best across all servo positions
        best = max(self.position_results, key=lambda x: x['max_area'])
        frame = best['frame']
        detections = best['detections']
        location = best['location']

        # Annotate frame
        results = model(frame)[0]
        annotated = results.plot()
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            cv2.putText(annotated, str(area), (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Ensure plant folder exists
        plant_folder = os.path.join(save_dir_base, f"plant_{self.current_plant_id:03d}")
        os.makedirs(plant_folder, exist_ok=True)

        # Save best image
        img_name = f"best_frame_loc{location:02d}.jpg"
        img_path = os.path.join(plant_folder, img_name)
        if cv2.imwrite(img_path, annotated):
            rospy.loginfo(f"[YOLO Stream] Saved best frame to: {img_path}")
        else:
            rospy.logerr(f"[YOLO Stream] Failed to save best frame to: {img_path}")

        # Write CSV of detections
        csv_path = os.path.join(plant_folder, "detections.csv")
        write_header = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if write_header:
                writer.writerow([
                    "plant_id", "location",
                    "plant_type", "bug_present",
                    "flower_present", "avg_confidence",
                    "max_area", "image_file"
                ])
            writer.writerow([
                self.current_plant_id,
                location,
                detections.get('plant_type', ''),
                int(detections.get('bug_present', False)),
                int(detections.get('flower_present', False)),
                round(sum(detections.get('confidences', [])) / len(detections.get('confidences', [1])), 3),
                best['max_area'],
                img_name
            ])

    def _next_position(self):
        if self.index >= len(self.positions):
            rospy.loginfo(f"[YOLO Stream] Completed all positions for plant {self.current_plant_id}, loc {self.current_location}")
            # save summary for entire location
            self.save_location_summary()
            # reset servo to home position 1500µs
            rospy.loginfo("[YOLO Stream] Resetting servo to 1500 µs before done")
            servo_pub.publish(ServoPulseWidth(channel=3, pulse_width_in_microseconds=1500))
            rospy.sleep(1.0)
            self.done_pub.publish(Bool(data=True))
            return
        self.collecting = True
        # start fresh capture buffer
        self.raw_frames = []
        # start a timeout to skip this angle if no frames collected
        if self.timeout_timer:
            self.timeout_timer.shutdown()
        self.timeout_timer = rospy.Timer(
            rospy.Duration(10.0),  # 10-second timeout per position
            self._position_timeout,
            oneshot=True
        )
        # Additional logic for moving servo and resetting collection would be here
        pulse_width = self.positions[self.index]
        servo_msg = ServoPulseWidth(channel=3, pulse_width_in_microseconds=pulse_width)
        servo_pub.publish(servo_msg)
        rospy.loginfo(f"[YOLO Stream] Moved servo to {pulse_width} μs for plant_id {self.current_plant_id}, location {self.current_location}")
        rospy.sleep(2)  # allow servo to reach position
        self.reset()
        self.collecting = True

    def _position_timeout(self, event):
        with self.lock:
            if self.collecting:
                rospy.logwarn(f"[YOLO Stream] Timeout: no good frames at pulse {self.positions[self.index]}")
                self.collecting = False
                # advance to next
                self.index += 1
                self._next_position()

plant_collector = PlantCollector()

def at_plant_callback(msg):
    if msg.plant_id == -1:
        rospy.loginfo("[YOLO Stream] Received reset (plant_id=-1); stopping collection.")
        # stop any ongoing collection
        plant_collector.collecting = False
        if plant_collector.timeout_timer:
            plant_collector.timeout_timer.shutdown()
        return
    rospy.loginfo(f"[YOLO Stream] Received /asc/at_plant message for plant_id {msg.plant_id}")
    plant_collector.done_pub.publish(Bool(data=False))
    rospy.loginfo(f"[YOLO Stream] Beginning frame collection for plant_id {msg.plant_id}")
    plant_collector.start_collection(msg.plant_id, msg.location)

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