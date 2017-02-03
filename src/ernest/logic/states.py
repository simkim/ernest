#!/usr/bin/python
import rospy
import time
import threading
from dynamixel_msgs.msg import MotorState
from ernest.srv import StringAndResult, ServoAngle
from smach import State


class PrototypeState(State):
    def wait(self, sleep_time, msg):
        rospy.loginfo(msg)
        time.sleep(sleep_time)


class Sleeping(PrototypeState):
    def __init__(self):
        State.__init__(self, outcomes=['sensing'], output_keys=['interest'])
        self.sleep_condition = threading.Condition()

    def wakeup(self):
        self.sleep_condition.acquire()
        self.sleep_condition.notify()
        try:
            self.head_subscriber.unregister()
        except:
            pass
        self.sleep_condition.release()

    def check_head(self, req):
        if req.load > 20 or req.load < -20:
            rospy.loginfo("Some load applied to head, wake up")
            self.wakeup()

    def request_preempt(self):
        self.wakeup()
        State.request_preempt(self)

    def execute(self, ud):
        self.sleep_condition.acquire()
        self.head_subscriber = rospy.Subscriber(
            "/ernest_body/motors/head", MotorState, self.check_head)
        self.sleep_condition.wait()
        ud['interest'] = 2
        self.sleep_condition.release()
        return 'sensing'


class Greet(PrototypeState):
    def __init__(self):
        State.__init__(self, outcomes=['success'], io_keys=['interest'])
        self.set_head_angle = rospy.ServiceProxy(
            'ernest_body/set_head_angle', ServoAngle)
        self.say = rospy.ServiceProxy('ernest_mouth/say', StringAndResult)

    def execute(self, ud):
        rospy.loginfo("Greeting")
        self.set_head_angle(10)
        self.say("Salut")
        self.set_head_angle(-10)
        ud['interest'] -= 1
        return 'success'


class DoSomething(PrototypeState):
    def __init__(self):
        State.__init__(self, outcomes=['greet', 'failure'])

    def execute(self, ud):
        self.wait(1, "What could I do ?")
        return 'greet'


class DoNothing(PrototypeState):
    def __init__(self):
        State.__init__(self, outcomes=['success'], io_keys=['interest'])

    def execute(self, ud):
        self.wait(3, "Doing nothing ... ")
        ud['interest'] -= 1
        return 'success'


class ActiveState(State):
    """ Main loop """
    def __init__(self):
        State.__init__(self,
                       outcomes=['action', 'timeout', 'bored'],
                       input_keys=['interest'])

    def execute(self, ud):
        if ud['interest'] <= 0:
            rospy.loginfo("Nothing interesting, I'm bored")
            return 'bored'
        if ud['interest'] <= 1:
            rospy.loginfo("Nothing interesting")
            return 'timeout'
        rospy.loginfo("Will do something")
        return 'action'
