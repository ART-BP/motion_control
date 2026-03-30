#!/usr/bin/env python3
import random

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

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

def generate_random_point(current_pose, min_xy, max_xy):
    x = random.gauss(current_pose[0], (max_xy - min_xy) / 4)
    y = random.gauss(current_pose[1], (max_xy - min_xy) / 4)
    return (x, y)

def check_point_in_bounds(point, min_xy, max_xy):
    return min_xy <= point[0] <= max_xy and min_xy <= point[1] <= max_xy

def main():
    rospy.init_node("path_random_publisher")

    path_topic = rospy.get_param("~path_topic", "/path")
    frame_id = rospy.get_param("~frame_id", "odom")
    num_send = int(rospy.get_param("~num_send", 3))

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
    while not rospy.is_shutdown() and num_send > 0:
        path_msg = Path()
        num_points = random.randint(min_points, max_points)
        for _ in range(num_points):
            while True:
                random_point = generate_random_point(pose, min_xy, max_xy)
                if check_point_in_bounds(random_point, min_xy, max_xy):
                    break
            pose_msg = build_pose(random_point[0], random_point[1], rospy.Time.now(), frame_id)
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = frame_id
            path_msg.poses.append(pose_msg)
            pose = random_point
        pub.publish(path_msg)
        rospy.loginfo("Published random path with %d points", num_points)
        num_send -= 1
        rospy.sleep(5.0)
    
    
if __name__ == "__main__":
    main()
