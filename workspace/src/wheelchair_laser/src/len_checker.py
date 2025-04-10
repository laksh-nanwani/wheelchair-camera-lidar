#!/usr/bin/env python3

import rospy
from livox_ros_driver.msg import CustomMsg


def callback(msg: CustomMsg):
    rospy.loginfo(f"Received {len(msg.points)} points.")


if __name__ == "__main__":
    rospy.init_node("len_checker", anonymous=True)
    rospy.Subscriber("livox/lidar", CustomMsg, callback)
    while not rospy.is_shutdown():
        rospy.spin()
