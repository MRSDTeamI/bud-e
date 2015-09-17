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

#############################################################
# This is the controller for our Crustcrawler AX-12A arm.
#############################################################

import roslib
#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
from std_msgs.msg import Bool
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

        rospy.Subscriber("shoulder_pan_joint/state", JointState, self.shoulder_pan_callback)
        rospy.Subscriber("shoulder_pitch_joint/state", JointState, self.shoulder_pitch_callback)
       
        rospy.sleep(1)  # so we have some time to get pan_angle from the callback

        self.at_home = False
        self.move_home()   # self.at_home will be "True" after this call

    def move_joint(self, angles):

        ## If arm is at home, move it up first to move out of the way of the base
        #if self.at_home:
        #    # [shoulder pan, shoulder_pitch, elbow, wrist, gripper]
        #    self.move_joint([self.pan_angle, -0.5, 1.7, 0, -1.7])  # open gripper to widest
        #    self.at_home = False   # arm is no longer at home position

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_pitch_joint','elbow_flex_joint','wrist_roll_joint','gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)                       
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

    def shoulder_pan_callback(self, data):
        self.pan_angle = data.current_pos

    def shoulder_pitch_callback(self, data):
        self.pitch_angle = data.current_pos

    def move_home(self, vision=False):
        '''
        Move the arm to a 'home' position.
        If vision module == True: 'home' is the down and to the right (starboard side of 
        the robot). This allows us to (1) be closer to the table/bottle, and (2) not block 
        the Kinect.
        If vision module == False: 'home' is upright and infront of the robot. This
        allows us to (1) keep the arm from bumping into objects while navigating, and (2)
        not have the scissor lift list to one side.

        '''
        gripper_neutral = -1

        # Vision module started, 'home' is to the side.
        if vision == True:
            shoulder_pan = -1.5     # pan arm to rightside of robot
            shoulder_pitch = 0   # arm flared out
            elbow = 1.7             # flexed (90 deg)

            # Lift arm up at elbow 
            self.move_joint([self.pan_angle, self.pitch_angle, elbow, 0, gripper_neutral]) 
            # Rotate to right side
            self.move_joint([shoulder_pan, self.pitch_angle, elbow, 0, gripper_neutral])
            # Put arm down on right side
            self.move_joint([shoulder_pan, shoulder_pitch, elbow, 0, gripper_neutral])
            
            ## Move elbow up to level first
            #self.move_joint([self.pan_angle, shoulder_pitch, elbow, 0, gripper_neutral])
            ## Move should_pan to the side
            #self.move_joint([shoulder_pan, shoulder_pitch, elbow, 0, gripper_neutral])
            ## Move elbow down to default position
            #self.move_joint([shoulder_pan, -1, elbow, 0, gripper_neutral])
        # Vision module not started, 'home' is in front
        else:
            shoulder_pan = 0        # pan arm to front of robot
            shoulder_pitch = 1.7    # arm upright
            elbow = -1.7            # flexed (90 deg)
            # Move elbow up above base first
            self.move_joint([self.pan_angle, shoulder_pitch, elbow, 0, gripper_neutral])
            # Move should_pan so arm is facing the front
            self.move_joint([shoulder_pan, shoulder_pitch, elbow, 0, gripper_neutral])
       
        self.at_home = True 

def main():
        arm = Joint()
        arm.move_joint([-1.0,1.9,0.0,0,-1])
        #arm.move_joint([0.5,1.5,1.0])
        #arm.move_joint([6.28,3.14,6.28])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
