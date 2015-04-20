#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
    def __init__(self):
        self.jta = actionlib.SimpleActionClient('/Arm/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

            
    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()                      
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_pitch_joint','elbow_flex_joint','wrist_roll_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)                       
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)
              

def main():
        arm = Joint()
        arm.move_joint([-1.0,1.9,0.0,0])
        #arm.move_joint([0.5,1.5,1.0])
        #arm.move_joint([6.28,3.14,6.28])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
