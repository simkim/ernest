#!/usr/bin/python
from os import system
from ernest.srv import StringAndResult
import rospy


def say(req):
	print "Say %s" % req.string
	system("pico2wave -l fr-FR -w /tmp/ernest_say.wav \"%s\"" % req.string)
	system("play -q /tmp/ernest_say.wav gain -0.15 pitch -200.0 gain -0.1 tempo -s 1.6")
	return "ok"

def main():
	rospy.init_node('ernest_mouth')
	s = rospy.Service('ernest_mouth/say', StringAndResult, say)
	rospy.spin()

if __name__ == "__main__":
	main()
