#!/usr/bin/env python3
import random

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import math

def liner_fun(x):
    return x

def sqrt_fun(x):
    return x ** 0.5

def square_fun(x):
    return x ** 2

def cube_fun(x):
    return x ** 3

def exp_fun(x):
    return math.exp(-x)

fanctions = [liner_fun, sqrt_fun, square_fun, cube_fun, exp_fun]


def build_pose(x, y, stamp, frame_id):
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

def callback(msg, pose):
    pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)


def main():
    rospy.init_node("path_random_publisher")

    path_topic = rospy.get_param("~path_topic", "path")
    frame_id = rospy.get_param("~frame_id", "odom")
    num_send = int(rospy.get_param("~num_send", 5))

    min_points = int(rospy.get_param("~min_points", 5))
    max_points = int(rospy.get_param("~max_points", 20))
    min_xy = float(rospy.get_param("~min_xy", -15.0))
    max_xy = float(rospy.get_param("~max_xy", 15.0))

    min_points = max(1, min(min_points, 20))
    max_points = max(1, min(max_points, 20))
    if min_points > max_points:
        min_points, max_points = max_points, min_points

    if min_xy > max_xy:
        min_xy, max_xy = max_xy, min_xy

    pub = rospy.Publisher(path_topic, Path, queue_size=1)
    pose = (0.0, 0.0)


    rospy.Subscriber("/odom", Odometry, callback, pose, queue_size=1)
    rospy.wait_for_message("/odom", Odometry, timeout=10)
    while not rospy.is_shutdown():
        function = random.choice(fanctions)
        num_points = random.randint(min_points, max_points)
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = rospy.Time.now()
        x = 0.0
        for i in range(num_points):
            x += random.uniform(0.3, 0.7)
            y = function(x) + random.uniform(-0.1, 0.1)
            if x > max_xy or y > max_xy or x < min_xy or y < min_xy:    
                break
            path.poses.append(build_pose(x, y, path.header.stamp, frame_id))

        num_send -= 1
        if num_send <= 0:
            rospy.loginfo("Finished sending paths.")
            break
        pub.publish(path)
        rospy.loginfo(f"Published path with {len(path.poses)} points using function {function.__name__}")
        rospy.sleep(1.0)

if __name__ == "__main__":
    main()
