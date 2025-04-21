#!/usr/bin/env python3

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, Imu  # , PointField
from sensor_msgs import point_cloud2


class Livox_Laser_to_Pointcloud:
    def __init__(self) -> None:
        xfer_format = rospy.get_param("/xfer_format", default=0)

        if xfer_format == 0:
            self.pub_scan = rospy.Publisher("/livox/lidar", PointCloud2, queue_size=10)
            self.sub_scan = rospy.Subscriber("/livox/inverted_lidar", PointCloud2, self.pointcloud2_callback)

        elif xfer_format == 1:
            self.pub_scan = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=10)
            self.sub_scan = rospy.Subscriber("/livox/inverted_lidar", CustomMsg, self.customMsg_callback)

        else:
            raise NotImplementedError(f"Method undefined for xfer_format = {xfer_format}")

        self.pub_imu = rospy.Publisher("/livox/imu", Imu, queue_size=10)
        self.sub_imu = rospy.Subscriber("/livox/inverted_imu", Imu, self.imu_callback)

    def pointcloud2_callback(self, msg: PointCloud2):
        points = []
        for point in point_cloud2.read_points(msg, skip_nans=True):
            point = list(point)
            point[1] = -point[1]
            point[2] = -point[2]
            points.append(point)

        pointcloud_msg = point_cloud2.create_cloud(msg.header, msg.fields, points)

        self.pub_scan.publish(pointcloud_msg)

    def customMsg_callback(self, msg: CustomMsg):
        # points = []
        for p in msg.points:
            p.y = -p.y
            p.z = -p.z

        self.pub_scan.publish(msg)

    def imu_callback(self, msg: Imu):
        msg.angular_velocity.y = -msg.angular_velocity.y
        msg.angular_velocity.z = -msg.angular_velocity.z

        self.pub_imu.publish(msg)


if __name__ == "__main__":
    rospy.init_node("Livox_Laser_to_Pointcloud")
    Livox_Laser_to_Pointcloud()

    while not rospy.is_shutdown():
        rospy.spin()
