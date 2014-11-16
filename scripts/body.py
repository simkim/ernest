#!/usr/bin/python
import rospy
import actionlib
from ernest.msg import SetJointPositionAction, SetJointPositionGoal
from ernest.srv import ServoAngle
from servo.config import ServoConfig
from servo.network import ServoNetwork

DEG90=307
DEG45=int(DEG90/2)
P180=512
P90=P180-DEG90
P270=P180+DEG90
HEAD_FORWARD=int(P180-DEG90/2)
HEAD_MAX=HEAD_FORWARD-10

def ax_clamp_angle(angle):
	return max(min(angle, 150), -150)

def ax_clamp_position(position):
	return max(min(position, 1023), 0)

class DynamixelJointServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer("SetJointPosition", SetJointPositionAction, self.execute, False)
		self.server.start()
		self.config  = ServoConfig("ernest.ini")
		self.network = ServoNetwork(self.config)
		self.head = self.network.get_servo_by_label("head")
		self.head.synchronized = False
		self.head.max_torque = 1023
		self.head.torque_limit = 1023
		self.head.moving_speed = 0
		self.head.goal_position = HEAD_FORWARD
		self.head.torque_enable = 1
	def execute(self, goal):
		self.do()
		self.server.set_succeeded()
	def do(self, goal):
		print goal
		print goal.joints
		self.head.goal_position = HEAD_FORWARD + HEAD_MAX
		import time
		time.sleep(1)
		self.head.goal_position = HEAD_FORWARD - HEAD_MAX
	def set_head_angle(self, req):
		print "set_head_angle : %s" % req.angle
		angle = ax_clamp_angle(req.angle)
		self.head.goal_position = ax_clamp_position(HEAD_FORWARD+1024/300*angle)
		return 0
		
def main():
	rospy.init_node('ernest_body')
	server = DynamixelJointServer()
	s = rospy.Service('ernest_body/set_head_angle', ServoAngle, server.set_head_angle)
	sub = rospy.Subscriber("/ernest_body/goal", SetJointPositionGoal, server.do)
	rospy.spin()

if __name__ == "__main__":
	main()
