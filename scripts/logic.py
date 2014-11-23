#!/usr/bin/python
from smach import State, StateMachine
import smach_ros
import rospy
import time
import threading
from dynamixel_msgs.msg import MotorState
from ernest.srv import StringAndResult, ServoAngle

class PrototypeState(State):
	def wait(self, sleep_time, msg):
		rospy.loginfo(msg)
		time.sleep(sleep_time)
		

class Sleeping(PrototypeState):
	def __init__(self):
		State.__init__(self, outcomes=['sensing'], output_keys=['interest'])
		self.sleep_condition = threading.Condition()
	def check_head(self, req):
		self.sleep_condition.acquire()
		if req.load > 20 or req.load < -20:
			rospy.loginfo("Some load applied to head, wake up")
			self.sleep_condition.notify()
			self.head_subscriber.unregister()
		self.sleep_condition.release()	
	def request_preempt(self):
		self.sleep_condition.acquire()
		self.sleep_condition.notify()
		self.sleep_condition.release()
		State.request_preempt(self)
	def execute(self, ud):
		self.sleep_condition.acquire()
		self.head_subscriber = rospy.Subscriber("/ernest_body/motors/head", MotorState, self.check_head)
		self.sleep_condition.wait()
		ud['interest'] = 2
		self.sleep_condition.release()
		return 'sensing'

class Greet(PrototypeState):
	def __init__(self):
		State.__init__(self, outcomes=['success'], io_keys=['interest'])
		self.set_head_angle = rospy.ServiceProxy('ernest_body/set_head_angle', ServoAngle)
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
	
class WaitState(State):
	""" Main loop """
	def __init__(self):
		State.__init__(self, outcomes=['action', 'timeout', 'bored'], input_keys=['interest'])
	def execute(self, ud):
		if ud['interest'] <= 0:
			rospy.loginfo("Nothing interesting, I'm bored")
			return 'bored'
		if ud['interest'] <= 1:
			rospy.loginfo("Nothing interesting")
			return 'timeout'
		rospy.loginfo("Will do something")
		return 'action'

def create_ernest_sm():
	""" Create Ernest State Machine """

	awake_sm = StateMachine(
		outcomes=['bored'], 
		input_keys=['interest']
	)	

	with awake_sm:
		StateMachine.add('WAIT',         WaitState(),   transitions={'action' : 'DO_SOMETHING', 'timeout' : 'DO_NOTHING', 'bored' : 'bored'})
		StateMachine.add('DO_SOMETHING', DoSomething(), transitions={'greet': 'GREET', 'failure': 'DO_NOTHING'})
		StateMachine.add('DO_NOTHING',   DoNothing(),   transitions={'success': 'WAIT'})
		StateMachine.add('GREET',        Greet(),       transitions={'success' : 'WAIT'})

	root_sm = StateMachine(
		outcomes=[]
	)

	with root_sm:
		StateMachine.add('SLEEPING',     Sleeping(),    transitions={'sensing': 'AWAKE'})
		StateMachine.add('AWAKE',        awake_sm,      transitions={'bored': 'SLEEPING'})
	return root_sm

def main():
	import rospy
	rospy.init_node('ernest_logic')
	sm = create_ernest_sm()
	sis = smach_ros.IntrospectionServer('ernest_logic', sm, '/SM_ROOT')
	sis.start()
	smach_thread = threading.Thread(target=sm.execute)
	smach_thread.start()
	# Wait for ctrl-c
	rospy.spin()
	print "ctrl-c"
	# Request the container to preempt
	sm.request_preempt()
	sis.stop()

	# Block until everything is preempted 
	# (you could do something more complicated to get the execution outcome if you want it)
	smach_thread.join()

if __name__ == '__main__':
	main()
