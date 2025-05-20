#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from datetime import datetime
import glob

plant_id = 0

# Global buffer for latest frame
bridge = CvBridge()
model = YOLO("/home/asc/ASClinic/catkin_ws/src/asclinic_pkg/src/plant_detector/weights.pt")

def ros_callback(msg):
    try:
        global plant_id
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = model(cv_image)[0]
        if results.boxes and len(results.boxes) > 0:
            plant_folder = os.path.join("/home/asc/saved_camera_images/saved_plant_images", f"plant_{plant_id:03d}")
            os.makedirs(plant_folder, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_path = os.path.join(plant_folder, f"plant_{timestamp}.jpg")
            annotated_image = results.plot()
            cv2.imwrite(image_path, annotated_image)

            csv_path = os.path.join(plant_folder, "detections.csv")
            with open(csv_path, "a") as f:
                for box in results.boxes:
                    cls = int(box.cls)
                    conf = float(box.conf)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    class_name = model.names[cls]
                    f.write(f"{timestamp},{class_name},{conf:.2f},{area:.1f}\n")

            plant_id += 1
    except Exception as e:
        rospy.logwarn(f"[YOLO Stream] Failed to process frame: {e}")

def main():
    save_dir = "/home/asc/saved_camera_images/saved_plant_images/"
    os.makedirs(save_dir, exist_ok=True)
    for f in glob.glob(os.path.join(save_dir, "*.jpg")):
        os.remove(f)

    rospy.init_node("yolo_stream_node", anonymous=True)
    rospy.Subscriber("/camera_image", Image, ros_callback, queue_size=1, buff_size=2**24)
    rospy.loginfo("[YOLO Stream] Subscribed to /camera_image")

    rospy.spin()

if __name__ == '__main__':
    main()