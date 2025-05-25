#!/usr/bin/env python3

import rospy
from asclinic_pkg.msg import PlantDetection
from std_msgs.msg import Bool

current_state = {'plant_id': 1, 'location': 1, 'step': 0}

def plant_done_callback(msg):
    if msg.data:
        pub = rospy.Publisher('/asc/at_plant', PlantDetection, queue_size=1)
        rospy.sleep(1.0)  # ensure publisher is registered

        # signal that we are no longer at any plant
        off_msg = PlantDetection()
        off_msg.are_at_plant = False
        off_msg.plant_id = -1
        off_msg.location = -1
        pub.publish(off_msg)
        rospy.loginfo("[Test Plant] Published at_plant=False (reset)")

        rospy.loginfo(f"[Test Plant] Received plant_done=True for step {current_state['step']}")
        rospy.sleep(3.0)

        current_state['step'] += 1

        if current_state['step'] == 1:
            current_state['location'] = 2
            current_state['plant_id'] = 1
#        elif current_state['step'] == 2:
#            current_state['location'] = 1
#            current_state['plant_id'] = 2
#        elif current_state['step'] == 3:
#            current_state['location'] = 2
#            current_state['plant_id'] = 2
        else:
            rospy.loginfo("[Test Plant] All locations tested. Beginning inference.")
            # trigger batch inference
            batch_pub = rospy.Publisher('/batch_inference', Bool, queue_size=1)
            rospy.sleep(1.0)  # ensure publisher is registered
            batch_pub.publish(Bool(data=True))
            rospy.loginfo("[Test Plant] Published /batch_inference=true")
            rospy.signal_shutdown("Test complete.")
            return

        msg = PlantDetection()
        msg.are_at_plant = True
        msg.plant_id = current_state['plant_id']
        msg.location = current_state['location']
        pub.publish(msg)
        rospy.loginfo(f"[Test Plant] Published /asc/at_plant for plant_id {msg.plant_id}, location {msg.location}")
        
def main():
    rospy.init_node('test_plant_publisher')
    pub = rospy.Publisher('/asc/at_plant', PlantDetection, queue_size=1)
    rospy.Subscriber('/asc/plant_done', Bool, plant_done_callback)
    rospy.sleep(1.0)  # wait for publisher to register
    msg = PlantDetection()
    msg.are_at_plant = True
    msg.plant_id = 1
    msg.location = 1
    pub.publish(msg)
    rospy.loginfo("[Test Plant] Published initial /asc/at_plant message for plant_id 1, location 1")
    rospy.spin()
if __name__ == '__main__':
    main()