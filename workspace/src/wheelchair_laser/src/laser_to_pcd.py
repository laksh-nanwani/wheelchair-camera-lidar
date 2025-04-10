#!/usr/bin/env python3

import rospy
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


class Livox_Laser_to_Pointcloud:
    def __init__(self) -> None:
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.pub = rospy.Publisher("/livox/pcd", PointCloud2, queue_size=10)

        self.sub = rospy.Subscriber("/livox/lidar", CustomMsg, self.process_livox_custom_msg)

    def process_livox_custom_msg(self, msg: CustomMsg):
        points = []
        for p in msg.points:
            points.append([p.x, p.y, p.z, p.reflectivity])

        pointcloud_msg = point_cloud2.create_cloud(msg.header, self.fields, points)
        self.pub.publish(pointcloud_msg)


if __name__ == "__main__":
    rospy.init_node("Livox_Laser_to_Pointcloud")
    Livox_Laser_to_Pointcloud()

    while not rospy.is_shutdown():
        rospy.spin()
