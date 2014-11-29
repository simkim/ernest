#!/usr/bin/python
import rospy
from sensor_msgs.msg import Joy
from ernest.srv import ServoAngle


class Teleop:
    def __init__(self):
        rospy.wait_for_service('ernest_body/set_head_angle')
        try:
            self.set_head_angle = rospy.ServiceProxy(
                'ernest_body/set_head_angle', ServoAngle)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        self.last_x = 0

    def joy_info(self, joy):
        if abs(self.last_x - joy.axes[0]) < 0.10:
            return
        self.last_x = joy.axes[0]
        try:
            self.set_head_angle(90*joy.axes[0])
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


def main():
    rospy.init_node('ernest_teleop')
    teleop = Teleop()
    rospy.Subscriber('ernest_sensors/joy', Joy, teleop.joy_info)
    rospy.spin()

if __name__ == "__main__":
    main()
