#!/usr/bin/env python3

import threading

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

STATE_NO_READY = "NO_READY"
STATE_READY = "READY"
STATE_GOAL_REACHED = "GOAL_REACHED"
STATE_GOAL_NOT_REACHED = "GOAL_NOT_REACHED"
STATE_GOAL_TIMEOUT = "GOAL_TIMEOUT"

ALL_STATES = {
    STATE_NO_READY,
    STATE_READY,
    STATE_GOAL_REACHED,
    STATE_GOAL_NOT_REACHED,
    STATE_GOAL_TIMEOUT,
}

class CitywalkerService(object):
    def __init__(self):
        self.status_service = rospy.get_param("~status_service", "/citywalker/get_goal_status")
        self.wait_timeout_sec = float(rospy.get_param("~wait_timeout_sec", 0.0))

        self._cv = threading.Condition()
        self.state = STATE_NO_READY
        self.status_srv = rospy.Service(self.status_service, SetBool, self._status_callback)
        rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        with self._cv:
            self._cv.notify_all()

    def set_state(self, state):
        state = state.strip().upper()
        if state not in ALL_STATES:
            rospy.logwarn("Invalid citywalker state: %s", state)
            return False
        with self._cv:
            self.state = state
            self._cv.notify_all()
        return True

    def _status_callback(self, req):
        if req.data:
            if self.state == STATE_READY:
                self.set_state(STATE_GOAL_NOT_REACHED)
                return SetBoolResponse(success=True, message=STATE_READY)
            else:
                return SetBoolResponse(success=False, message=self.state)

        with self._cv:
            if self.state == STATE_NO_READY or self.state == STATE_READY:
                return SetBoolResponse(success=False, message=self.state)

            predicate = lambda: rospy.is_shutdown() or self.state in {STATE_GOAL_REACHED, STATE_GOAL_TIMEOUT}
            if self.wait_timeout_sec > 0.0:
                done = self._cv.wait_for(predicate, timeout=self.wait_timeout_sec)
                if not done and self.state == STATE_GOAL_NOT_REACHED:
                    self.state = STATE_GOAL_TIMEOUT
            else:
                self._cv.wait_for(predicate)

            return SetBoolResponse(
                success=True,
                message=self.state,
            )

def main():
    rospy.init_node("citywalker_service")
    CitywalkerService()
    rospy.spin()


if __name__ == "__main__":
    main()
