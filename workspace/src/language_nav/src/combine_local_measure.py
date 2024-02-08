#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Float64MultiArray, Float64

class CombinedROSPublisher:
    def __init__(self):
        rospy.init_node('combined_ros_publisher', anonymous=True)

        # Publishers
        self.covariance_pub = rospy.Publisher('/covariance_matrix', Float64MultiArray, queue_size=10)
        self.ess_pub = rospy.Publisher('/amcl_ess_estimate', Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber('/particlecloud', PoseArray, self.particlecloud_callback)

    def amcl_callback(self, msg):
        # Extract and publish the covariance matrix
        covariance_matrix = msg.pose.covariance
        covariance_msg = Float64MultiArray()
        covariance_msg.data = covariance_matrix
        self.covariance_pub.publish(covariance_msg)

    def particlecloud_callback(self, data):
        # Calculate and publish ESS estimate
        ess_estimate = self.calculate_ess_estimate(data)
        self.ess_pub.publish(ess_estimate)

        # Calculate and log NND
        nnd = self.compute_nnd([pose.position for pose in data.poses])
        rospy.loginfo("Average Nearest Neighbor Distance: %f", nnd)

    def calculate_ess_estimate(self, particles):
        x_positions = [p.position.x for p in particles.poses]
        y_positions = [p.position.y for p in particles.poses]
        var_x = np.var(x_positions)
        var_y = np.var(y_positions)
        return 1.0 / ((var_x + var_y) / 2.0)

    def compute_nnd(self, particles):
        if len(particles) < 2:
            return float('inf')
        min_distances = []
        for i, particle in enumerate(particles):
            min_distance = float('inf')
            for j, other_particle in enumerate(particles):
                if i != j:
                    distance = math.sqrt((particle.x - other_particle.x) ** 2 + (particle.y - other_particle.y) ** 2)
                    min_distance = min(min_distance, distance)
            min_distances.append(min_distance)
        return sum(min_distances) / len(min_distances)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    combined_ros_publisher = CombinedROSPublisher()
    combined_ros_publisher.run()
