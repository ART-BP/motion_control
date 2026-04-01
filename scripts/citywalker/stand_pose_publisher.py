#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

STANCE = {
    "FL_hip_joint": 0.08,
    "FR_hip_joint": -0.08,
    "RL_hip_joint": 0.08,
    "RR_hip_joint": -0.08,
    "FL_thigh_joint": 0.85,
    "FR_thigh_joint": 0.85,
    "RL_thigh_joint": 1.00,
    "RR_thigh_joint": 1.00,
    "FL_calf_joint": -1.55,
    "FR_calf_joint": -1.55,
    "RL_calf_joint": -1.70,
    "RR_calf_joint": -1.70,
}


def main():
    rospy.init_node("stand_pose_publisher")
    publish_rate = rospy.get_param("~publish_rate", 30.0)
    duration = rospy.get_param("~duration", 10.0)

    publishers = {}
    for joint, target in STANCE.items():
        topic = "/{}_position_controller/command".format(joint)
        publishers[joint] = (rospy.Publisher(topic, Float64, queue_size=1), target)

    # Give controller spawner and topic registration a moment.
    rospy.sleep(1.0)

    rate = rospy.Rate(publish_rate)
    start_time = rospy.Time.now()
    try:
        while not rospy.is_shutdown():
            for _, (pub, target) in publishers.items():
                pub.publish(Float64(data=target))

            if duration > 0.0 and (rospy.Time.now() - start_time).to_sec() >= duration:
                break
            rate.sleep()

        if duration > 0.0:
            rospy.loginfo("Stand pose command published for %.1f seconds", duration)
        else:
            rospy.loginfo("Stand pose command publishing stopped by shutdown")
    except rospy.ROSInterruptException:
        pass
if __name__ == "__main__":
    main()
