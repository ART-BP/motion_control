#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from actionlib_msgs.msg import GoalStatus


class VisualNavActionServer:
    def __init__(self):
        self.server_name = rospy.get_param("~server_name", "/citywalker_goal")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.3)

        self.latest_odom = None
        self.latest_image = None

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        self.server = actionlib.SimpleActionServer(
            self.server_name,
            MoveBaseAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("VisualNav action server started: %s", self.server_name)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def image_cb(self, msg):
        self.latest_image = msg

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def distance_to_goal(self, gx, gy):
        if self.latest_odom is None:
            return float("inf")
        px = self.latest_odom.pose.pose.position.x
        py = self.latest_odom.pose.pose.position.y
        return math.hypot(gx - px, gy - py)

    def run_visual_model(self, goal):
        """
        你的视觉导航模型入口：
        输入：goal + 最新图像/里程计
        输出：vx, wz
        """
        # TODO: 替换成你的模型推理逻辑
        # 示例：简单比例控制（占位）
        gx = goal.target_pose.pose.position.x
        gy = goal.target_pose.pose.position.y
        if self.latest_odom is None:
            return 0.0, 0.0

        px = self.latest_odom.pose.pose.position.x
        py = self.latest_odom.pose.pose.position.y
        dx, dy = gx - px, gy - py
        vx = max(min(0.5 * math.hypot(dx, dy), 0.3), 0.0)
        wz = 0.0
        return vx, wz

    def execute_cb(self, goal):
        rospy.loginfo("Receive goal: frame=%s x=%.3f y=%.3f",
                      goal.target_pose.header.frame_id,
                      goal.target_pose.pose.position.x,
                      goal.target_pose.pose.position.y)

        feedback = MoveBaseFeedback()
        result = MoveBaseResult()
        rate = rospy.Rate(10)

        gx = goal.target_pose.pose.position.x
        gy = goal.target_pose.pose.position.y

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self.stop_robot()
                self.server.set_preempted(result, "preempted")
                return

            dist = self.distance_to_goal(gx, gy)
            if dist < self.goal_tolerance:
                self.stop_robot()
                self.server.set_succeeded(result, "goal reached by visual nav")
                return

            vx, wz = self.run_visual_model(goal)

            cmd = Twist()
            cmd.linear.x = vx
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)

            if self.latest_odom is not None:
                feedback.base_position.header.stamp = rospy.Time.now()
                feedback.base_position.header.frame_id = self.latest_odom.header.frame_id
                feedback.base_position.pose = self.latest_odom.pose.pose
                self.server.publish_feedback(feedback)

            rate.sleep()

        self.stop_robot()
        self.server.set_aborted(result, "ros shutdown")


if __name__ == "__main__":
    rospy.init_node("citywalker_goalservice")
    VisualNavActionServer()
    rospy.spin()

