#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import os

def talker(filepath):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        f = open(filepath, "r")
        hello_str = f.read() 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    path = os.path.join("/".join(__file__.split("/")[:6]), "commands.txt")
    
    try:
        talker(path)
    except rospy.ROSInterruptException:
        pass
