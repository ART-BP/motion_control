#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node("unpause_when_ready")
    wait_for = rospy.get_param("~wait_for", 2.0)
    service_name = rospy.get_param("~service_name", "/gazebo/unpause_physics")

    if wait_for > 0.0:
        rospy.sleep(wait_for)

    try:
        rospy.loginfo("Waiting for service: %s", service_name)
        rospy.wait_for_service(service_name, timeout=30.0)
        unpause = rospy.ServiceProxy(service_name, Empty)
        unpause()
        rospy.loginfo("Gazebo physics unpaused.")
    except (rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("Failed to unpause Gazebo physics: %s", str(exc))


if __name__ == "__main__":
    main()
