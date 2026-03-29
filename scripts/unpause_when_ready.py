#!/usr/bin/env python3
import time

import rospy
from std_srvs.srv import Empty


def main():
    rospy.init_node("unpause_when_ready")
    wait_for = float(rospy.get_param("~wait_for", 2.0))
    service_name = str(rospy.get_param("~service_name", "/gazebo/unpause_physics"))

    if wait_for > 0.0:
        # Use wall time to avoid blocking forever when /use_sim_time=true and Gazebo is paused.
        time.sleep(wait_for)

    candidate_services = [service_name]
    if service_name != "/unpause_physics":
        candidate_services.append("/unpause_physics")

    for srv in candidate_services:
        try:
            rospy.loginfo("Waiting for service: %s", srv)
            rospy.wait_for_service(srv, timeout=30.0)
            unpause = rospy.ServiceProxy(srv, Empty)
            unpause()
            rospy.loginfo("Gazebo physics unpaused via %s", srv)
            return
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Unpause via %s failed: %s", srv, str(exc))

    rospy.logerr("Failed to unpause Gazebo physics. Checked services: %s", candidate_services)


if __name__ == "__main__":
    main()
