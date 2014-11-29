#!/usr/bin/python
from ernest.srv import SetMood, GetMood
from ernest.brain import Brain
import rospy

ernest_brain = Brain()


def set_mood(req):
    print "Set mood to %s" % req.mood
    ernest_brain.set_mood(req.mood)
    return "ok"


def get_mood(req):
    return ernest_brain.mood


def main():
    rospy.init_node('ernest_brain')
    rospy.Service('ernest_brain/set_mood', SetMood, set_mood)
    rospy.Service('ernest_brain/get_mood', GetMood, get_mood)
    rospy.spin()

if __name__ == "__main__":
    main()
