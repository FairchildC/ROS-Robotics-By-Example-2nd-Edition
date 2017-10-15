#!/usr/bin/env python

# Origional code from Rethink Robotics, Copyright (c) 2013-2015  
# This code has been modified encapsulate Baxter's arms in a class and 
# provide methods to control the arms.
# It works with the YMCAStateMach.py

# Refer to ROS Robotics By Example 2nd edition for a detailed
# explanation of this software.

import argparse

from copy import deepcopy

import rospy

from std_msgs.msg import (
    Empty,
    Bool,
)

import baxter_interface

from baxter_core_msgs.msg import (
    CollisionAvoidanceState,
)
from baxter_interface import CHECK_VERSION


class Baxter_Arms():
    def __init__(self):
        rospy.loginfo("Initializing node now... ")
        rospy.init_node("moving_arms")
        rospy.loginfo("Node Initialized. ")
        self._done = False
        self._limbs = ('left', 'right')
        self._arms = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right'),
            }
        self._pose_rate = rospy.Rate(20.0)  # Hz
        self._pose_threshold = 0.2   # radians
        self._peak_angle = -1.6      # radians
        self._arm_state = {
                           'pose': {'left': 'none', 'right': 'none'},
                           'collide': {'left': False, 'right': False},
                           'flipped': {'left': False, 'right': False}
                          }

     	  #Empty positional library that will be fed in and overwritten as arms_pose.
        self._joint_moves = {
            'letter': {
                       'left':  [0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0], 
                       'right':  [0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0]
                       } 
            
                         #DoF Key [s0,s1,e0,e1,w0,w1,w2]
            }
        self._collide_lsub = rospy.Subscriber(
                             'robot/limb/left/collision_avoidance_state',
                             CollisionAvoidanceState,
                             self._update_collision, 'left')
        self._collide_rsub = rospy.Subscriber(
                             'robot/limb/right/collision_avoidance_state',
                             CollisionAvoidanceState,
                             self._update_collision, 'right')
        self._disable_pub = {
            'left': rospy.Publisher(
                 'robot/limb/left/suppress_collision_avoidance',
                 Empty, queue_size=10),
            'right': rospy.Publisher(
                 'robot/limb/right/suppress_collision_avoidance',
                 Empty, queue_size=10)
        }
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._enable_pub = rospy.Publisher('robot/set_super_enable', 
                                           Bool, queue_size=10)

    def _update_collision(self, data, limb):
        self._arm_state['collide'][limb] = len(data.collision_object) > 0
        self._check_arm_state()

    def _check_arm_state(self):
        """
        Check for goals and behind collision field.

        If s1 joint is over the peak, collision will need to be disabled
        to get the arm around the head-arm collision force-field.
        """
        diff_check = lambda a, b: abs(a - b) <= self._pose_threshold
        for limb in self._limbs:
            angles = [self._arms[limb].joint_angle(joint)
                      for joint in self._arms[limb].joint_names()]

            # Check if in a goal position
            letter_goal = map(diff_check, angles,
                            self._joint_moves['letter'][limb])                      
            if all(letter_goal):
                self._arm_state['pose'][limb] = 'letter' 
            else:
                self._arm_state['pose'][limb] = 'none'

            # Check if shoulder is flipped over peak
            self._arm_state['flipped'][limb] = (
                self._arms[limb].joint_angle(limb + '_s1') <= self._peak_angle)

    def _move_to(self, new_pose, disabled):
        if any(disabled.values()):
            [pub.publish(Empty()) for pub in self._disable_pub.values()]
        while (any(self._arm_state['pose'][limb] != goal
                   for limb, goal in new_pose.viewitems())
               and not rospy.is_shutdown()):
            if self._rs.state().enabled == False:
                self._enable_pub.publish(True)
            for limb in self._limbs:
                if disabled[limb]:
                    self._disable_pub[limb].publish(Empty())
                if limb in new_pose:
                    self._arms[limb].set_joint_positions(dict(zip(
                                      self._arms[limb].joint_names(),
                                      self._joint_moves[new_pose[limb]][limb])))
            self._check_arm_state()
            self._pose_rate.sleep()

        if any(self._arm_state['collide'].values()):
            self._rs.disable()
        return


    #Importing data in the form of arms_pose, origionally paired with YMCAStateMach.py 
    def supervised_move(self, arms_pose):
        self._joint_moves = arms_pose    # Updating private variable with new arms_pose
        
        self._check_arm_state()
        #Update our starting state to check if arms are posed?
        rospy.loginfo("Movement in progress.")
        suppress = deepcopy(self._arm_state['flipped'])
        actions = {'left': 'letter', 'right': 'letter'}    
        self._move_to(actions, suppress)
        self._done = True
        return

    def clean_shutdown(self):
        """Handles ROS shutdown (Ctrl-C) safely."""
        if not self._done:
            rospy.logwarn('Aborting: Shutting down safely...')
        if any(self._arm_state['collide'].values()):
            while self._rs.state().enabled != False:
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
                self._enable_pub.publish(False)
                self._pose_rate.sleep()


def main():
    barms = Baxter_Arms()
    rospy.on_shutdown(barms.clean_shutdown)
    barms.supervised_move()
    rospy.loginfo("Finished move.")
    
if __name__ == "__main__":
    main()
