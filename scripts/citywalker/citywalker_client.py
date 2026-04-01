#!/usr/bin/env python3

import argparse
import sys

import rospy
from std_srvs.srv import SetBool


def _parse_args():
    parser = argparse.ArgumentParser(description="Client for /citywalker/get_goal_status (SetBool)")
    parser.add_argument(
        "--service",
        default="/citywalker/get_goal_status",
        help="Service name (default: /citywalker/get_goal_status)",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for service. <=0 means wait forever.",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--reset",
        action="store_true",
        help="Call service with data=true, reset state to NO_GOAL.",
    )
    group.add_argument(
        "--wait",
        action="store_true",
        help="Call service with data=false, wait until GOAL_REACHED or GOAL_TIMEOUT.",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = _parse_args()
    request_data = False
    if args.reset:
        request_data = True
    elif args.wait:
        request_data = False

    rospy.init_node("citywalker_client", anonymous=True)

    try:
        if args.connect_timeout > 0.0:
            rospy.wait_for_service(args.service, timeout=args.connect_timeout)
        else:
            rospy.wait_for_service(args.service)
    except rospy.ROSException as exc:
        rospy.logerr("Wait service timeout: %s", exc)
        return 2

    proxy = rospy.ServiceProxy(args.service, SetBool)
    try:
        resp = proxy(request_data)
    except rospy.ServiceException as exc:
        rospy.logerr("Service call failed: %s", exc)
        return 3

    print("success={success}, message={message}".format(
        success=str(resp.success).lower(),
        message=resp.message,
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
