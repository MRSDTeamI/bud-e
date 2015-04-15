#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
# Author: Acorn Pooley
# Author: Johnny Wang

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

# For debugging purposes
debug = 0;
joint_vals = 1;  # Take joint values (4) as input or x,y,z for pose

class budeArmCommander():

  def __init__(self):
    ## init moveit_commander and rospy.
    print "============ Starting tutorial setup"
  
    moveit_commander.roscpp_initialize(sys.argv)
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    self.robot = moveit_commander.RobotCommander()
    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in BUD-E's arm
    #self.group = moveit_commander.MoveGroupCommander("arm")
    self.group = moveit_commander.MoveGroupCommander("Arm")
    #self.group.allow_replanning(True)
    self.group.set_goal_tolerance(0.1)
    self.group.set_planner_id("RRTkConfigDefault")
    #self.group.set_planner_id("SBLkConfigDefault")

    if joint_vals:
        rospy.Subscriber("bottle_center", String, self.callback)
    else:
        rospy.Subscriber("bottle_center", geometry_msgs.msg.Vector3, self.callback)

    while not rospy.is_shutdown():
      rospy.sleep(1)

  def __del__(self):
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

  # end_coord is a geometry/Vector3 object
  def callback(self, end_coord):
    if joint_vals:
        rospy.loginfo(rospy.get_caller_id() + " ^^^^  %s", end_coord)
    else:
        rospy.loginfo(rospy.get_caller_id() + " ^^^^  %f %f %f", end_coord.x, end_coord.y, end_coord.z)

    if debug:
      self.print_joint_state()
      #self.print_end_effector_pose()
      #x = input("x: ")
      #y = input("y: ")
      #z = input("z: ")
      #end_coord = Vector3(x,y,z)

    self.generate_plan(end_coord)

    #if debug:
    #  self.generate_plan(Vector3(-0.025, -1.277, 0.6))
    #  self.generate_plan(Vector3(0.284, 0.096, 0.032))
  
  def print_joint_state(self):
    print "============ Printing robot joint state"
    print self.robot.get_current_state()
    print "============"

  def print_end_effector_pose(self):
    print "============ Printing robot end-effector pose"
    print self.group.get_current_pose()
    print "============"

  # Generate a pose to the end coordinate
  # INPUT:
  #    end_coord is a geometry/Vector3 object
  def generate_plan(self, end_coord):

    self.group.set_goal_position_tolerance(0.01)
    self.group.set_goal_orientation_tolerance(0.01)
    self.group.set_pose_reference_frame("base")

    end_eff = self.group.get_end_effector_link()
    #print end_eff

    if joint_vals:
        print type(end_coord.data)
        new_coord = str.split(end_coord.data)
        print new_coord
        """
        Joint variable names:
          arm_base_joint
          shoulder_pan_joint
          shoulder_pitch_joint
          elbow_flex_joint
          wrist_roll_joint

        Arm straight up joint values:
          -0.2198  1.7  1.242  -0.1891
        """
        print len(new_coord)
        self.group.set_joint_value_target([float(new_coord[0]), float(new_coord[1]), 
            float(new_coord[2]), float(new_coord[3])])
        print self.group.get_joints()
        print self.group.get_current_joint_values()   

    else:
        ps = self.group.get_current_pose()
        ps.pose.position.x = end_coord.x
        ps.pose.position.y = end_coord.y
        ps.pose.position.z = end_coord.z
        #ps = geometry_msgs.msg.Pose()
        #ps.position.x = end_coord.x
        #ps.position.y = end_coord.y
        #ps.position.z = end_coord.z
        #ps.orientation.w = 1

        #ps.pose.position.x = 0.2404
        #ps.pose.position.y = 0.2130
        #ps.pose.position.z = 0.1802
        #ps.pose.orientation.x = -0.0290
        #ps.pose.orientation.y = -0.0219
        #ps.pose.orientation.z = 0.3469
        #ps.pose.orientation.w = 0.93716
        #new_pose = self.group.get_random_pose(end_eff)
        #print "---- Random Pose ----"
        #print new_pose

        #new_pose = self.group.get_current_pose()
        #print "---- Current Pose ----"
        #print new_pose
        #print self.group.get_planning_frame()
        #print type(ps.pose)

        #print self.group.get_pose_reference_frame()
        #self.group.set_pose_target(new_pose)

        #self.group.set_position_target([end_coord.x, end_coord.y, end_coord.z], end_eff)
        #self.group.set_position_target([end_coord.x, end_coord.y, end_coord.z]) 

        #self.group.set_pose_target(ps)

    #print self.group.get_goal_tolerance()

  	## Now, we call the planner to compute the plan
  	## and visualize it if successful
  	## Note that we are just planning, not asking move_group
  	## to actually move the robot
    if debug:
      print "============ Generating plan"
      print "x: %f" % end_coord.x
      print "y: %f" % end_coord.y
      print "z: %f" % end_coord.z

    my_plan = self.group.plan()

    rospy.sleep(5)
    self.group.go(wait=True)

    print "Joint values AFTER:"
    print self.group.get_current_joint_values()

    print "DONE"

if __name__=='__main__':

  rospy.init_node('moveit_cmd_listener', anonymous=True)

  try:
    arm_cmd = budeArmCommander()
  except rospy.ROSInterruptException:
    pass
