#!/usr/bin/env python3
import random

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import math


class PathRandomPublisher:
    def __init__(self):
        rospy.init_node("path_random_publisher")

        self.power = (0.1, 0.2, 0.5, 0.8, 1.0, 1.2, 1.5, 1.8, 2.0, 2.2, 2.5, 2.8, 3.0)
        self.path_topic = rospy.get_param("~path_topic", "path")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.num_path = int(rospy.get_param("~num_path", 5))
        self.orient = int(rospy.get_param("~orient", 0))
        self.min_points = int(rospy.get_param("~min_points", 4))
        self.max_points = int(rospy.get_param("~max_points", 8))
        self.min_xy = float(rospy.get_param("~min_xy", -15.0))
        self.max_xy = float(rospy.get_param("~max_xy", 15.0))

        self.min_points = max(1, min(self.min_points, 20))
        self.max_points = max(1, min(self.max_points, 20))
        if self.min_points > self.max_points:
            self.min_points, self.max_points = self.max_points, self.min_points

        if self.min_xy > self.max_xy:
            self.min_xy, self.max_xy = self.max_xy, self.min_xy

        self.pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscribers to connect to the path topic...")
            rospy.sleep(0.5)

        self.pose = (0.0, 0.0, 0.0)

        rospy.Subscriber("/odom", Odometry, self.callback, queue_size=1)
        rospy.wait_for_message("/odom", Odometry, timeout=10)
        
    def build_pose(self, x, y, stamp, frame_id):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def local_to_global(self, local_pose, current_pose):
        local_x, local_y = local_pose
        ref_x, ref_y, yaw = current_pose
        global_x = ref_x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        global_y = ref_y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
        return (global_x, global_y)

    def callback(self, msg):
        yaw = math.atan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z), 1.0 - 2.0 * (msg.pose.pose.orientation.z ** 2))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def generate_random_path(self, orient, pose, num_points, function_index, frame_id):
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = rospy.Time.now()
        path.poses.append(self.build_pose(pose[0], pose[1], path.header.stamp, frame_id))
        local_x = 0.0
        for _ in range(num_points):
            local_x += random.uniform(0.3, 0.6)
            local_y = (local_x ** function_index + random.uniform(-0.1, 0.1)) * orient

            x, y = self.local_to_global((local_x, local_y), pose)
            if x > self.max_xy or y > self.max_xy or x < self.min_xy or y < self.min_xy:    
                break
            path.poses.append(self.build_pose(x, y, path.header.stamp, frame_id))
        return path

    def main(self):
        rate = rospy.Rate(0.2)  # Publish at 0.2 Hz (every 5 seconds)
        while not rospy.is_shutdown():
            num_points = random.randint(self.min_points, self.max_points)
            function_index = random.choice(self.power)
            orient = random.choice([-1, 1]) if self.orient == 0 else self.orient
            path = self.generate_random_path(orient, self.pose, num_points, function_index, self.frame_id)
            self.pub.publish(path)
            self.num_path -= 1
            print(f"Publishing path, remaining: {self.num_path}")
            if self.num_path <= 0:
                break
            rate.sleep()

if __name__ == "__main__":
    prp = PathRandomPublisher()
    prp.main()
