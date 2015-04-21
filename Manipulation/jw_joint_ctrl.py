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
from dynamixel_msgs.msg import JointState


class Joint:
    def __init__(self):
        self.jta = actionlib.SimpleActionClient('/Arm/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

        rospy.Subscriber("shoulder_pan_joint/state", JointState, self.callback)
       
        rospy.sleep(1)  # so we have some time to get pan_angle from the callback
 
        self.move_home()

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()                      
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_pitch_joint','elbow_flex_joint','wrist_roll_joint','gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)                       
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

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
        # Move elbow_flex up to level first
        self.move_joint([self.pan_angle, -0.5, 1.7, 0, -1])
        # Move should_pan to the side
        self.move_joint([-1.4, -0.5, 1.7, 0, -1])
        # Move elbow down to default position
        self.move_joint([-1.4, -1.4, 1.7, 0, -1])
        

def main():
        arm = Joint()
        arm.move_joint([-1.0,1.9,0.0,0,-1])
        #arm.move_joint([0.5,1.5,1.0])
        #arm.move_joint([6.28,3.14,6.28])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
