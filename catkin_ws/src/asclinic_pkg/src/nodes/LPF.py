import rospy
from asclinic_pkg.msg import PoseCovar

#!/usr/bin/env python

class LowPassFilterNode:
    def __init__(self):
        rospy.init_node('lpf_node', anonymous=True)
        
        # Get filter parameter (alpha between 0 and 1) with default value 0.2
        self.alpha = rospy.get_param('~alpha', 0.2)
        
        # Threshold on y variance
        self.epsilon = rospy.get_param('~y_variance_epsilon', 0.1)
        # Last valid y value (within variance bound)
        self.last_valid_y = None
        
        self.pub = rospy.Publisher("/pose_LPF", PoseCovar, queue_size=10)
        self.sub = rospy.Subscriber("/pose_estimate_fused", PoseCovar, self.callback)
        
    def callback(self, msg):
        # Build output message
        out = PoseCovar()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = msg.header.frame_id

        # Always forward x and phi and their covariances
        out.x = msg.x
        out.phi = msg.phi
        out.xvar = msg.xvar
        out.phivar = msg.phivar
        out.xycovar = msg.xycovar
        out.xphicovar = msg.xphicovar
        out.yphicovar = msg.yphicovar

        # Conditionally forward y based on its variance
        if msg.yvar <= self.epsilon:
            out.y = msg.y
            self.last_valid_y = msg.y
        else:
            # If no prior valid y, fall back to current
            out.y = self.last_valid_y if self.last_valid_y is not None else msg.y
        out.yvar = msg.yvar

        # Publish filtered message
        self.pub.publish(out)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LowPassFilterNode()
    node.run()