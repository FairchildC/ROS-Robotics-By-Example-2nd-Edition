#!/usr/bin/env python

# Using the ROS SMACH package, this Python script creates five states 
# corresponding to Baxter's arm poses for each letter Y, M, C, A and a 
# fifth state for a neutral pose. When one pose of the arms completes, 
# the state will successfully complete and the next state will begin. 

# Refer to ROS Robotics By Example 2nd edition for a detailed
# explanation of this software.

import rospy
from smach import State,StateMachine

from time import sleep
from MoveControl import Baxter_Arms

class Y(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.letter_y = {
            'letter': {
                      'left':  [0.0, -1.0, 0.0, 0.0,  0.0, 0.0, 0.0], 
                      'right':  [0.0, -1.0, 0.0, 0.0,  0.0, 0.0, 0.0]
                       } }
                         #DoF Key [s0,s1,e0,e1,w0,w1,w2]

    def execute(self, userdata):
        rospy.loginfo('Give me a Y!')
        barms.supervised_move(self.letter_y)
        sleep(2)
        return 'success'

class M(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.letter_m = {
            'letter': {
                      'left':  [0.0, -1.50, 1.0, -0.052,  3.0, 2.094, 0.0], 
                      'right':  [0.0, -1.50, -1.0, -0.052,  -3.0, 2.094, 0.0]
                       } }
                         #DoF Key [s0,s1,e0,e1,w0,w1,w2]

    def execute(self, userdata):
        rospy.loginfo('Give me a M!')
        barms.supervised_move(self.letter_m)
        sleep(2)
        return 'success'

class C(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.letter_c = {
            'letter': {
                      'left':  [0.80, 0.0, 0.0, -0.052,  3.0, 1.50, 0.0], 
                      'right':  [0.0, -1.50, -1.0, -0.052, -3.0, 1.0, 0.0]
                       } } 
                         #DoF Key [s0,s1,e0,e1,w0,w1,w2]

    def execute(self, userdata):
        rospy.loginfo('Give me a C!')
        barms.supervised_move(self.letter_c)
        sleep(2)
        return 'success'

class A(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.letter_a = {
            'letter': {
                      'left':  [0.50, -1.0, -3.0, 1.0, 0.0, 0.0, 0.0], 
                      'right':  [-0.50, -1.0, 3.0, 1.0,  0.0, 0.0, 0.0]
                       } } 
                         #DoF Key [s0,s1,e0,e1,w0,w1,w2]

    def execute(self, userdata):
        rospy.loginfo('Give me an A!')
        barms.supervised_move(self.letter_a)
        sleep(2)
        return 'success'


class Zero(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.zero = {
            'letter': {
                      'left':  [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00], 
                      'right':  [0.00, 0.00, 0.00, 0.00,  0.00, 0.00, 0.00]
                       } } 
                         #DoF Key [s0,s1,e0,e1,w0,w1,w2]

    def execute(self, userdata):
        rospy.loginfo('Ta-da')
        barms.supervised_move(self.zero)
        sleep(2)
        return 'success'

if __name__ == '__main__':

    barms = Baxter_Arms()
    rospy.on_shutdown(barms.clean_shutdown)

    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('Y', Y(), transitions={'success':'M'})
        StateMachine.add('M', M(), transitions={'success':'C'})
        StateMachine.add('C', C(), transitions={'success':'A'})
        StateMachine.add('A', A(), transitions={'success':'ZERO'})
        StateMachine.add('ZERO', Zero(), transitions={'success':'success'})  

    sm.execute()

