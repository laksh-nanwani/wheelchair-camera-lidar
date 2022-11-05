#!/usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

LINEAR_THRESHOLD = 0.1
ANGULAR_THRESHOLD = 0.05

LINEAR_VEL = 0.2
ANGULAR_VEL = 0.3

class Benchmark:
	def __init__(self) -> None:
		self.init_publishers()
		self.init_subscribers()

		self.robot_pose_x = 0
		self.robot_pose_y = 0
		self.robot_heading_angle = 0

	def init_subscribers(self):
		rospy.loginfo("initializing subscribers")
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

	def init_publishers(self):
		rospy.loginfo("initializing publishers")
		self.cmd_vel_pub = rospy.Publisher("/wheelchair_diff/cmd_vel", Twist, queue_size = 10)

	def odom_callback(self, odom_msg):
		self.robot_pose_x = odom_msg.pose.pose.position.x
		self.robot_pose_y = odom_msg.pose.pose.position.y

		heading_quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
		self.robot_heading_angle = euler_from_quaternion(heading_quat)[2]
		# rospy.loginfo(f"Pose :: x : {self.robot_pose_x:.2f} ; y : {self.robot_pose_y:.2f} ; heading : {self.robot_heading_angle:.2f}")


	def move_forward(self, distance):
		start_x = self.robot_pose_x
		start_y = self.robot_pose_y
		start_theta = self.robot_heading_angle

		target_x = start_x + distance * math.cos(start_theta)
		target_y = start_y + distance * math.sin(start_theta)
		target_theta = start_theta

		rospy.loginfo("move_forward")
		rospy.loginfo(f"start  :: x : {start_x:.2f} m ; y : {start_y:.2f} m ; heading : {start_theta:.2f} rad")
		rospy.loginfo(f"target :: x : {target_x:.2f} m ; y : {target_y:.2f} m ; heading : {target_theta:.2f} rad")
		rospy.loginfo("--- starting ---")

		target_reached = False # False
		tic = time.time()
		while (not target_reached):
			err = math.sqrt((self.robot_pose_x - target_x)**2 + (self.robot_pose_y - target_y)**2)
			rospy.loginfo(f"Pose :: x : {self.robot_pose_x:.2f} m ; y : {self.robot_pose_y:.2f} m ; heading : {self.robot_heading_angle:.2f} rad  :: Error : {err:.2f}")

			if err < LINEAR_THRESHOLD:
				target_reached = True
				continue


			# TODO: pid using err, for now its gonna be 1 or 0
			linear = LINEAR_VEL
			angular = 0
			self.control_robot(linear, angular)
			time.sleep(0.1)

		toc = time.time()  
		self.control_robot(0, 0)
		rospy.loginfo("--- target reached ---")
		rospy.loginfo(f"time_taken : {toc - tic:.2f}")


	def rotate(self, angle):
		start_theta = self.robot_heading_angle
		target_theta = self.fold_angle(start_theta + angle)

		rospy.loginfo("rotate")
		rospy.loginfo(f"start_angle  :: {start_theta:.2f}")
		rospy.loginfo(f"target_angle :: {target_theta:.2f}")
		rospy.loginfo("--- starting ---")
		
		tic = time.time()

		target_reached = False
		while not target_reached:
			err = self.fold_angle(self.robot_heading_angle - target_theta)
			rospy.loginfo(f"Heading :: current : {self.robot_heading_angle:.2f} m ; target : {target_theta:.2f} m ; error : {err:.2f} rad")
			
			if abs(err) < ANGULAR_THRESHOLD:
				target_reached = True
				continue


			# TODO: pid using err, for now its gonna be 1 or 0
			linear = 0
			angular = ANGULAR_VEL
			self.control_robot(linear, angular)
		toc = time.time()  
		self.control_robot(0, 0)
		rospy.loginfo("--- target reached ---")
		rospy.loginfo(f"time_taken : {toc - tic:.2f}")

	def control_robot(self, linear, angular):
		cmd_vel = Twist()
		cmd_vel.linear.x = linear
		cmd_vel.angular.z = angular
		self.cmd_vel_pub.publish(cmd_vel)

	def emergency_stop(self):
		self.control_robot(0, 0)

	def fold_angle(self, angle):
		if angle > math.pi:
			angle -= 2 * math.pi
		elif angle < -math.pi:
			angle += 2 * math.pi
		return angle
		
	def benchmarking_loop(self):
		while not rospy.is_shutdown():
			self.move_forward(5)
			self.rotate(180)


if __name__ == "__main__":
	rospy.init_node("benchmark", anonymous=False)
	
	benchmark = Benchmark()
	rospy.on_shutdown(benchmark.emergency_stop)
	time.sleep(2)
	# benchmark.move_forward(2)
	benchmark.rotate(-3.14)
	# rospy.spin()