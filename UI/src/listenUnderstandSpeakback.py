#!/usr/bin/env python


import roslib; roslib.load_manifest('BUDE_Speech')
import rospy
from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient

#POP
import actionlib 
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from subprocess import call


class TalkBack:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
          
        self.voice = rospy.get_param("~voice", "voice_us1_mbrola")
        self.wavepath = rospy.get_param("~wavepath", "")
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        rospy.sleep(1)
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)
        
        rospy.loginfo("Say one of the navigation commands...")

        # Subscribe to the recognizer output
        rospy.Subscriber('/recognizer/output', String, self.talkback)
	# POP Movebase
	locations = dict()
        
        locations['kitchen'] = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
	# Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
	# Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(20))
        
        rospy.loginfo("Connected to move base server")


        
    def talkback(self, msg):
        # Print the recognized words on the screen

        command=msg.data
      #POPstart

        if command == 'buddy':    
            rospy.loginfo(command)
	    self.soundhandle.say("Hi! How can I help you?",self.voice)
		
            
        if command=='buddy go':
 	    rospy.loginfo(command)
	    self.soundhandle.say("Where should I go?",self.voice)
	if command=='buddy kitchen':
 	    rospy.loginfo
	    self.soundhandle.say("Okay. I know I have to go to the kitchen BUT, I don't know yet where the kitchen is!",self.voice)
	if command=='buddy go to the kitchen':
 	    rospy.loginfo(command)
	    self.soundhandle.say("Okay. I am planning my path to the kitchen",self.voice)
	    # Set up the goal location
            self.goal = MoveBaseGoal()
           #self.goal.target_pose.pose = locations['kitchen']
	    self.goal.target_pose.pose.position.x=9.37
	    self.goal.target_pose.pose.position.y=-10.2
	    self.goal.target_pose.pose.position.z=0.0
	    self.goal.target_pose.pose.orientation.w=1.0
	    self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going
            rospy.loginfo("Going to: " + "kitchen")
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
	    state = self.move_base.get_state()
            while 1:
	        if state == GoalStatus.SUCCEEDED:
               	    rospy.loginfo("BUD-E reached its destination!!")
	            self.soundhandle.say("Hello folks, Please congratulate buddy!! She has reached the goal!!")
                    rospy.loginfo("State:" + str(state))
		    break
                else:
                    rospy.loginfo("Waiting for BUD-E to reach the goal... " )#+ str(goal_states[state]))
	  #  call(["rostopic"] ["pub"] ["/move_base_simple/goal"] ["geometry_msgs/PoseStamped"] ["'{header: {stamp: now, frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"]
	

    def cleanup(self):
        rospy.loginfo("Shutting down the node...")

if __name__=="__main__":
    rospy.init_node('talkback')
    try:
        TalkBack()
        rospy.spin()
    except:
        pass

