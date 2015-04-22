#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Johnny Wang
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Johnny Wang

import roslib
#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from dynamixel_msgs.msg import JointState


class Joint:
    def __init__(self):
        self.jta = actionlib.SimpleActionClient('/Arm/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

        rospy.Subscriber("shoulder_pan_joint/state", JointState, self.callback)
       
        rospy.sleep(1)  # so we have some time to get pan_angle from the callback

        self.at_home = False
        self.move_home()   # self.at_home will be "True" after this call

    def move_joint(self, angles):

        # If arm is at home, move it up first to move out of the way of the base
        if self.at_home:
            # [shoulder pan, shoulder_pitch, elbow, wrist, gripper]
            self.move_joint([self.pan_angle, -0.5, 1.7, 0, -1.7])  # open gripper to widest

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_pitch_joint','elbow_flex_joint','wrist_roll_joint','gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)                       
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)
        
        self.at_home = False   # arm is no longer at home position

    def callback(self, data):
        self.pan_angle = data.current_pos

    def move_home(self):
        '''
        Move to HOME joint angles of [-1.4, -1.4, 1.7, 0]. This is the starboard side of
        the robot. We need to start the arm on this side because it allows us to be closer
        to the table/bottle.
        Move in parts so we don't accidentally bump into restricted areas (corners
        of the base the arm is mounted to.

        '''
        elbow = 1.7             # flexed (90 deg)
        shoulder_pan = -1.5     # pan arm to rightside of robot
        shoulder_pitch = -0.5   # arm flared out
        gripper_neutral = -1
        # Move elbow_flex up to level first
        self.move_joint([self.pan_angle, shoulder_pitch, elbow, 0, gripper_neutral])
        # Move should_pan to the side
        self.move_joint([shoulder_pan, shoulder_pitch, elbow, 0, gripper_neutral])
        # Move elbow down to default position
        self.move_joint([shoulder_pan, -1, elbow, 0, gripper_neutral])
       
        self.at_home = True 

def main():
        arm = Joint()
        arm.move_joint([-1.0,1.9,0.0,0,-1])
        #arm.move_joint([0.5,1.5,1.0])
        #arm.move_joint([6.28,3.14,6.28])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
