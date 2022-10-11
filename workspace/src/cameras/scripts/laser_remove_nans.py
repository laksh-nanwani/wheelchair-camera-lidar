#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
import copy
import numpy as np

class Nan_Remover:
	def __init__(self):
		self.sub = rospy.Subscriber('/camera2/depth_scan', LaserScan, self.remove_nan)
		self.pub = rospy.Publisher('/nan_removed_scan', LaserScan, queue_size = 30)
		rospy.loginfo("Nan Remover Started publishing on topic '/nan_removed_scan'")

	def remove_nan(self, msg):
#		out_scan = copy.deepcopy(msg)
#		ranges = np.array(msg.ranges)
#		ranges = ranges[~np.isnan(ranges)]
#		out_scan.ranges = ranges
#		self.pub.publish(out_scan)
                msg.ranges = np.nan_to_num(msg.ranges).tolist()
                self.pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('Nan_Remover', anonymous=False)
	nan_rem = Nan_Remover()
	rospy.spin()
