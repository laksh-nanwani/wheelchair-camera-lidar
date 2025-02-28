#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import Twist, PoseStamped

# from move_base_msgs.msg import MoveBaseAction
from locomotor_msgs.msg import NavigateToPoseAction, NavigateToPoseGoal, NavigateToPoseActionGoal
from tf.transformations import euler_from_quaternion
import time


class GoalStopper:
    def __init__(self) -> None:
        self.client = SimpleActionClient("/locomotor/navigate", NavigateToPoseAction)

        self.cmd_vel = Twist()
        self.zero_vel = Twist()
        self.zero_vel.linear.x = 0
        self.zero_vel.angular.z = 0

        while not self.client.wait_for_server():
            print("Waiting for server")
            time.sleep(1)
            continue

        print("server connected")

        self.pub = rospy.Publisher("/wheelchair_diff/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/wheelchair_diff/cmd_vel_raw", Twist, self.vel_callback)
        # self.zero_vel = Twist()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        self.sub = rospy.Subscriber("/move_base_simple/goal", Twist, self.goal_callback)

    def vel_callback(self, msg: Twist):
        # rospy.loginfo(self.client.get_state())
        self.cmd_vel = msg

    def timer_callback(self, event):
        print(self.client.get_state())
        if self.client.get_state() in [GoalStatus.SUCCEEDED, GoalStatus.LOST, GoalStatus.ABORTED, GoalStatus.REJECTED]:
            # with open("status.txt", "w") as file:
            #     file.write(self.client.get_state())
            self.pub.publish(self.zero_vel)
        else:
            self.pub.publish(self.cmd_vel)

    def goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id == "map":
            goal = NavigateToPoseGoal()
            goal.goal.pose.x = msg.pose.position.x
            goal.goal.pose.y = msg.pose.position.y

            theta = euler_from_quaternion(
                [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            )[2]
            goal.goal.pose.theta = theta
            goal.goal.header.frame_id = "map"
            self.client.send_goal(goal, feedback_cb=self.print_feedback)

        else:
            rospy.loginfo("Please provide goal in map frame")

    def print_feedback(self, feedback):
        pose = feedback.state.global_pose.pose
        vel = feedback.state.current_velocity.velocity
        print("%.2f %.2f %.2f | %.2f %.2f" % (pose.x, pose.y, pose.theta, vel.x, vel.theta))
        print("Global plan: %d poses" % len(feedback.state.global_plan.poses))
        print(
            "%.2f %.2f %.2f"
            % (feedback.percent_complete, feedback.distance_traveled, feedback.estimated_distance_remaining)
        )


if __name__ == "__main__":
    rospy.init_node("GoalStopper")
    GoalStopper()
    while not rospy.is_shutdown():
        rospy.spin()
