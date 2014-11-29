#!/usr/bin/python
import smach_ros
import rospy
import threading

from ernest.logic.statemachine import create_ernest_sm


def main():
    rospy.init_node('ernest_logic')
    sm = create_ernest_sm()

    # Start IntrospectionServer
    sis = smach_ros.IntrospectionServer('ernest_logic', sm, '/SM_ROOT')
    sis.start()

    # Start statemachine
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()
    print "ctrl-c"

    # Request the container to preempt
    sm.request_preempt()
    sis.stop()
    smach_thread.join()

if __name__ == "__main__":
    main()
