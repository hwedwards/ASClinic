#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import cv2
import os
import glob
from ultralytics import YOLO

# Path configuration
RAW_DIR = "/home/asc/saved_camera_images/saved_plant_images/raw_images"
PROCESSED_DIR = "/home/asc/saved_camera_images/saved_plant_images/processed_images"
MODEL_PATH = "/home/asc/ASClinic/catkin_ws/src/asclinic_pkg/src/plant_detector/weights_2.pt"

class BatchInferencer:
    def __init__(self):
        # Load model once
        self.model = YOLO(MODEL_PATH)
        # Subscribe to trigger
        rospy.Subscriber("/batch_inference", Bool, self.trigger_cb)
        rospy.loginfo("[BatchInference] Ready, waiting for /batch_inference")

    def trigger_cb(self, msg: Bool):
        if not msg.data:
            return
        rospy.loginfo("[BatchInference] Trigger received, running inference on raw images")
        # Ensure processed dir exists
        os.makedirs(PROCESSED_DIR, exist_ok=True)
        # Iterate all jpg/png in raw folder
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

                # Filter out low-confidence detections
                results = results.filter(conf=0.70)

                # Plot bounding boxes on the image
                annotated = results.plot()

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

def main():
    rospy.init_node("batch_inference_node", anonymous=True)
    BatchInferencer()
    rospy.spin()

if __name__ == "__main__":
    main()