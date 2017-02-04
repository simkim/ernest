#!/usr/bin/python
import rospy
import actionlib
from dynamixel_msgs.msg import MotorState
from ernest.msg import SetJointPositionAction, SetJointPositionGoal
from ernest.srv import ServoAngle
from servo.config import ServoConfig
from servo.network import ServoNetwork
import threading

DEG90 = 307
DEG45 = int(DEG90/2)
P180 = 512
P0 = 102
P90 = P180-DEG90
P270 = P180+DEG90
HEAD_FORWARD = int(P180-DEG90/2)
HEAD_MAX = HEAD_FORWARD-10
LSHOULDER_UP=int(237)
LSHOULDER_FRONT=int(P180)
RSHOULDER_UP=int(804)
RSHOULDER_FRONT=int(P180)


def ax_clamp_angle(angle):
    return max(min(angle, 150), -150)


def ax_clamp_position(position):
    return max(min(position, 1023), 0)


class DynamixelJointServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "SetJointPosition", SetJointPositionAction, self.execute, False)
        self.server.start()
        self.config = ServoConfig("ernest.ini")
        self.network = ServoNetwork(self.config)

        self.rshoulder = self.network.get_servo_by_label("rshoulder")
        self.rshoulder.synchronized = False
        self.rshoulder.max_torque = 1023
        self.rshoulder.torque_limit = 1023
        self.rshoulder.moving_speed = 0
        self.rshoulder.goal_position = RSHOULDER_FRONT
        self.rshoulder.torque_enable = 1
        self.lshoulder = self.network.get_servo_by_label("lshoulder")
        self.lshoulder.synchronized = False
        self.lshoulder.max_torque = 1023
        self.lshoulder.torque_limit = 1023
        self.lshoulder.moving_speed = 0
        self.lshoulder.goal_position = LSHOULDER_FRONT
        self.lshoulder.torque_enable = 1

        self.head = self.network.get_servo_by_label("head")
        self.head.synchronized = False
        self.head.max_torque = 1023
        self.head.torque_limit = 1023
        self.head.moving_speed = 0
        self.head.goal_position = HEAD_FORWARD
        self.head.torque_enable = 1
        self.head_load = rospy.Publisher(
            "/ernest_body/motors/head", MotorState, queue_size=1)
        self.lock = threading.RLock()

    def execute(self, goal):
        self.do()
        self.server.set_succeeded()

    def do(self, goal):
        print goal
        print goal.joints
        self.lock.acquire()
        self.head.goal_position = HEAD_FORWARD + HEAD_MAX
        self.lock.release()
        import time
        time.sleep(1)
        self.lock.acquire()
        self.head.goal_position = HEAD_FORWARD - HEAD_MAX
        self.lock.release()

    def set_head_angle(self, req):
        print "set_head_angle : %s" % req.angle
        angle = ax_clamp_angle(req.angle)
        self.lock.acquire()
        self.head.goal_position = ax_clamp_position(
            HEAD_FORWARD + 1024/300*angle)
        self.lock.release()
        return 0

    def set_rshoulder_angle(self, req):
        print "set_rshoulder_angle : %s" % req.angle
        angle = ax_clamp_angle(req.angle)
        self.lock.acquire()
        self.rshoulder.goal_position = ax_clamp_position(
            RSHOULDER_FRONT + 1024/300*angle)
        self.lock.release()
        return 0

    def set_lshoulder_angle(self, req):
        print "set_lshoulder_angle : %s" % req.angle
        angle = ax_clamp_angle(req.angle)
        self.lock.acquire()
        self.lshoulder.goal_position = ax_clamp_position(
            LSHOULDER_FRONT + 1024/300*angle)
        self.lock.release()
        return 0

    def loop(self):
        while not rospy.is_shutdown():
            ms = MotorState()
            self.lock.acquire()
            ms.load = self.head.current_load
            self.lock.release()
            self.head_load.publish(ms)
            rospy.Rate(50)


def main():
    rospy.init_node('ernest_body')
    server = DynamixelJointServer()
    rospy.Service('ernest_body/set_head_angle',
                  ServoAngle, server.set_head_angle)
    rospy.Service('ernest_body/set_rshoulder_angle',
                  ServoAngle, server.set_rshoulder_angle)
    rospy.Service('ernest_body/set_lshoulder_angle',
                  ServoAngle, server.set_lshoulder_angle)
    rospy.Subscriber("/ernest_body/goal", SetJointPositionGoal, server.do)
    try:
        server.loop()
    except:
        import sys
        sys.exit(-1)

if __name__ == "__main__":
    main()
