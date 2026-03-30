#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
import time

def enable_gazebo_physics():
    rospy.init_node('enable_gazebo_physics')
    delay = rospy.get_param('~delay', 2.0)
    rospy.loginfo("Waiting for Gazebo to start... Delaying for %.1f seconds", delay)
    time.sleep(delay)

    rospy.loginfo("Enabling Gazebo physics")
    try:
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=10)
        unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics()
        rospy.loginfo("Gazebo physics enabled")
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Failed to enable Gazebo physics: %s", e)

if __name__ == '__main__':
    enable_gazebo_physics()