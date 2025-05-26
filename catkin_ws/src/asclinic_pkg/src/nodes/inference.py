#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import cv2
import os
import glob
from ultralytics import YOLO

# Path configuration
RAW_DIR = "/home/asc/saved_camera_images/saved_plant_images/filtered_images"
PROCESSED_DIR = "/home/asc/saved_camera_images/saved_plant_images/processed_images"
MODEL_PATH = "/home/asc/ASClinic/catkin_ws/src/asclinic_pkg/src/plant_detector/weights_2.pt"

NUMBER_OF_LOCATIONS = 2  # Number of locations to wait for before triggering batch inference

class BatchInferencer:
    def __init__(self):
        # Load model once
        self.model = YOLO(MODEL_PATH)
        # Initialize counter for location_done messages
        self.location_done_count = 0
        # Subscribe to location_done to trigger inference after ten True signals
        rospy.Subscriber("/asc/location_done", Bool, self.location_done_cb)
        # Subscribe to trigger
        rospy.Subscriber("/batch_inference", Bool, self.trigger_cb)
        rospy.loginfo("[BatchInference] Ready, waiting for /batch_inference")

        # Clear processed images directory
        if os.path.exists(PROCESSED_DIR):
            for f in glob.glob(os.path.join(PROCESSED_DIR, "*")):
                os.remove(f)
            rospy.loginfo(f"[BatchInference] Cleared processed images in {PROCESSED_DIR}")
        else:
            os.makedirs(PROCESSED_DIR, exist_ok=True)
            rospy.loginfo(f"[BatchInference] Created processed images directory {PROCESSED_DIR}")

    def trigger_cb(self, msg: Bool):
        if not msg.data:
            return
        rospy.loginfo("[BatchInference] Trigger received, running inference on raw images")
        # Ensure processed dir exists
        os.makedirs(PROCESSED_DIR, exist_ok=True)
        # Iterate all jpg/png in filtered folder
        patterns = [os.path.join(RAW_DIR, "*.jpg"),
                    os.path.join(RAW_DIR, "*.png")]
                   
        for pattern in patterns:
            for raw_path in glob.glob(pattern):
                # Read image
                img = cv2.imread(raw_path)
                if img is None:
                    rospy.logwarn(f"[BatchInference] Failed to read {raw_path}")
                    continue

                # Run YOLO inference
                results = self.model(img)[0]

                # skip if no detections
                if not results.boxes or len(results.boxes) == 0:
                    rospy.loginfo(f"[BatchInference] No detections in {raw_path}, skipping")
                    continue

                # Draw bounding boxes with conf ≥ 0.70
                annotated = img.copy()
                for box in results.boxes:
                    conf = float(box.conf)
                    if conf < 0.70:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # draw rectangle
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # label text
                    cls = int(box.cls)
                    label = f"{results.names[cls]}:{conf:.2f}"
                    cv2.putText(annotated, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Construct processed path
                filename = os.path.basename(raw_path)
                proc_path = os.path.join(PROCESSED_DIR, filename)

                # Write out
                success = cv2.imwrite(proc_path, annotated)
                if success:
                    rospy.loginfo(f"[BatchInference] Saved processed image: {proc_path}")
                else:
                    rospy.logerr(f"[BatchInference] Failed to save: {proc_path}")

        rospy.loginfo("[BatchInference] Batch inference complete")

    def location_done_cb(self, msg: Bool):
        if not msg.data:
            return
        self.location_done_count += 1
        rospy.loginfo(f"[BatchInference] location_done True count: {self.location_done_count}/{NUMBER_OF_LOCATIONS}")
        if self.location_done_count >= NUMBER_OF_LOCATIONS:
            rospy.loginfo("[BatchInference] Finished all plant locations signals, running batch inference")
            # call trigger directly
            self.trigger_cb(Bool(data=True))
            # reset counter for next cycle
            self.location_done_count = 0

def main():
    rospy.init_node("batch_inference_node", anonymous=True)
    BatchInferencer()
    rospy.spin()

if __name__ == "__main__":
    main()