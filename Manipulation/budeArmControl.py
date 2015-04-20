#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

# For debugging purposes
debug = 1;

def budeArmCommander():

  ## init moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('budeArmCommander',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in BUD-E's arm
  group = moveit_commander.MoveGroupCommander("Arm")
  #group = moveit_commander.MoveGroupCommander("arm")
  group.set_goal_tolerance(0.01);
  group.set_planner_id("RRTkConfigDefault")
  #group.set_planner_id("SBLkConfigDefault")
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot joint state"
  print robot.get_current_state()
  print "============"

  print "============ Printing robot end-effector pose"
  print group.get_current_pose()
  print "============"
  ps  = group.get_current_pose()


  print "============ Generating plan 1"
  ps.pose.position.x= 0.25
  ps.pose.position.y= 0
  ps.pose.position.z= 0.5

  ps.pose.orientation.x= 0
  ps.pose.orientation.y= 0
  ps.pose.orientation.z= 0
  ps.pose.orientation.w= 1

  #ps.pose.position.x= 0.013
  #ps.pose.position.y= 0.0081
  #ps.pose.position.z= 0.066092524209

  #ps.pose.orientation.x= 0.501248595003
  #ps.pose.orientation.y= -0.49914727992
  #ps.pose.orientation.z= 0.500850951761
  #ps.pose.orientation.w= 0.498748597067


  #group.set_joint_value_target(ps,None,True)
  group.set_pose_target(ps)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group
  ## to actually move the robot
  plan1 = group.plan()
  group.execute(plan1)
  rospy.sleep(5)
  group.go(wait=True)


  if (debug):
    print "============ Generating plan 2"
    ps.pose.position.x= -0.0250002736296
    ps.pose.position.y= -1.27738778012e-05
    ps.pose.position.z= 0.600999087285

    ps.pose.orientation.x= 2.1157133798e-05
    ps.pose.orientation.y= -4.69467052624e-05
    ps.pose.orientation.z= 0.707069008992
    ps.pose.orientation.w= 0.707144549488

    group.set_joint_value_target(ps,None,True)

    ## Now, we call the planner to compute the plan
    ## and visualize it if successful
    ## Note that we are just planning, not asking move_group
    ## to actually move the robot
    plan1 = group.plan()

    rospy.sleep(5)
    group.go(wait=True)

    #print "============ Generating plan 3"
    #ps.pose.position.x=0.284533829625
    #ps.pose.position.y= 0.095949568364
    #ps.pose.position.z= 0.0320462807192

    #ps.pose.orientation.x= 0.115536326545
    #ps.pose.orientation.y= 0.963074963091
    #ps.pose.orientation.z= 0.104567923768
    #ps.pose.orientation.w= 0.219553005069

    #group.set_joint_value_target(ps,None,True)

    ### Now, we call the planner to compute the plan
    ### and visualize it if successful
    ### Note that we are just planning, not asking move_group
    ### to actually move the robot
    #plan1 = group.plan()

    #rospy.sleep(5)
    #group.go(wait=True)

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    budeArmCommander()
  except rospy.ROSInterruptException:
    pass

