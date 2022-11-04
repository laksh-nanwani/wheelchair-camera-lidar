#!/usr/bin/env python

import rospy
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Pub_Vel:
    def __init__(self):
        self.pub1 = rospy.Publisher("/lin_vel", Float32, queue_size = 20)
        self.pub2 = rospy.Publisher("/ang_vel", Float32, queue_size = 20)
        self.sub = rospy.Subscriber("/joint_state", JointState, self.publish_velocities)

    def publish_velocities(self, msg):
        vels = msg.velocity
        self.pub1.publish(vels[0])
        self.pub2.publish(vels[1])

if __name__ == "__main__":
    rospy.init_node("quat_to_euler", anonymous=False)
    Pub_Vel()
    rospy.spin()