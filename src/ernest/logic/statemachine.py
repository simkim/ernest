#!/usr/bin/python

from smach import StateMachine
from ernest.logic.states import ActiveState, DoNothing, DoSomething, Greet, ShakeHands
from ernest.logic.states import Sleeping


def create_awake_sm():
    """ Create awake State Machine, Ernest should act """
    awake_sm = StateMachine(
        outcomes=['bored'],
        input_keys=['interest']
    )

    with awake_sm:
        StateMachine.add('ACTIVE', ActiveState(),
                         transitions={'action': 'DO_SOMETHING',
                                      'timeout': 'DO_NOTHING',
                                      'bored': 'bored'})
        StateMachine.add('DO_SOMETHING', DoSomething(),
                         transitions={'greet': 'GREET',
                                      'shake_hands': 'SHAKE_HANDS',
                                      'failure': 'DO_NOTHING'})
        StateMachine.add('DO_NOTHING', DoNothing(),
                         transitions={'success': 'ACTIVE'})
        StateMachine.add('GREET', Greet(),
                         transitions={'success': 'ACTIVE'})
        StateMachine.add('SHAKE_HANDS', ShakeHands(),
                         transitions={'success': 'ACTIVE'})
    return awake_sm


def create_root_sm():
    """ Create root State Machine with meta state (sleeping, awake) """
    root_sm = StateMachine(
        outcomes=[]
    )

    with root_sm:
        StateMachine.add('SLEEPING', Sleeping(),
                         transitions={'sensing': 'AWAKE'})
        StateMachine.add('AWAKE', create_awake_sm(),
                         transitions={'bored': 'SLEEPING'})
    return root_sm


def create_ernest_sm():
    """ Create Ernest State Machine """
    return create_root_sm()
