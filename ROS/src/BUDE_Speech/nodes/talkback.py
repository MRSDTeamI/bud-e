#!/usr/bin/env python

"""
    talkback.py - 
"""

import roslib; roslib.load_manifest('BUDE_Speech')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt16

from sound_play.libsoundplay import SoundClient

#POP
import actionlib 
from actionlib_msgs.msg import *
#from actionlib_msgs.msg import GoalStatusArray
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
        #TEMP
        #state = self.move_base.get_state()
        #rospy.loginfo("\n\nTEMP Goal STATE= %s \n\n",state)

    def goBack(self):
        rospy.loginfo("Going back")
        self.soundhandle.say("I am going back.",self.voice)
	rospy.sleep(3)
        self.goal.target_pose.pose.position.x=-3.06
        self.goal.target_pose.pose.position.y=0.09
        self.goal.target_pose.pose.position.z=0.0
        self.goal.target_pose.pose.orientation.w=1.0
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal)

    def cmdScissorLift(self):
        pub = rospy.Publisher('target_height',UInt16, queue_size=rospy)
	pubKinect=rospy.Publisher('start_visiion',Bool,queue_size=rospy)
        #10.init_node('cmdScissorLift', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        #while not rospy.is_shutdown():
        heightCmd = 15
        rospy.loginfo("Sending %d to the target_height topic",heightCmd)
        pub.publish(heightCmd)
	pubKinect.publish(Bool(True))
        rospy.sleep(30.0)
        self.goBack()
        #rate.sleep()

    def getGoalStatus(self,goalMessage):
	 # Subscribe to the move_base client goal status
        rospy.loginfo("getGoalStatus called")
        rospy.loginfo(rospy.get_caller_id() + "Received %s", goalMessage)
        if goalMessage.status_list:
            curr_status = goalMessage.status_list[0].status
            #rospy.loginfo("Current Goal Status: %s",curr_status) 
	    if curr_status==3:
	        rospy.loginfo("BUD-E has reached the Goal")
	        self.soundhandle.say("I have reached my goal. Please congratulate me. Thanks you.",self.voice)
		rospy.sleep(3.0)
	        self.cmdScissorLift()

        
    def talkback(self, msg):
        # Print the recognized words on the screen
	
        command=msg.data
	# The following line should be moved below.
	rospy.Subscriber('move_base/status', GoalStatusArray,self.getGoalStatus)
      #POPstart

        if command == 'buddy':    
            rospy.loginfo(command)
	    self.soundhandle.say("Hi! How can I help you?",self.voice)
	
        if command=='buddy go':
 	    rospy.loginfo(command)
	    self.soundhandle.say("Where should I go?",self.voice)
	if command=='buddy bottle':
 	    rospy.loginfo(command)
	    self.soundhandle.say("Okay. I know I have to go to the kitchen BUT, I don't know yet where the kitchen is!",self.voice)
	if command=='buddy stop':
	    rospy.loginfo(command)
	    #rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}
	    self.goal =MoveBaseGoal()
	    self.goal.cancelGoal()
	if command=='buddy coffe':
	    rospy.loginfo(command)
	    self.soundhandle.say("Okay. I am trying to go high",self.voice)
	    self.cmdScissorLift()

	if command=='buddy get water':
 	    rospy.loginfo(command)
	    self.soundhandle.say("Okay. I am planning my path to the kitchen",self.voice)
	    # Set up the goal location
            self.goal = MoveBaseGoal()
           #self.goal.target_pose.pose = locations['kitchen']
	    self.goal.target_pose.pose.position.x=0.378
	    self.goal.target_pose.pose.position.y=-0.11
	    self.goal.target_pose.pose.position.z=0.0
	    self.goal.target_pose.pose.orientation.w=1.0
	    self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going
            rospy.loginfo("Going to: " + "kitchen")
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
	    state = self.move_base.get_state()
	    
		#rostopic echo /poseupdate
            #while 1:
            #actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED	
            	#rospy.loginfo("\n\nWhile loop Goal STATE= %s \n\n",state)
	        #if state == GoalStatus.SUCCEEDED:
		    #Command the scissorlift to rise to a height after BUD-E reaches the goal
		    	#try:
		    	#cmdScissorLift()
		    	#except rospy.ROSInterruptException:
		        #	pass

		    	#rospy.loginfo("BUD-E reached its destination!!")
	        	#self.soundhandle.say("Hello folks, Please congratulate buddy!! She has reached the goal!!")
	            #Going back to where BUD-E started
            		#rospy.loginfo("State:" + str(state))
		        #self.goal.target_pose.pose.position.x=0.00
			#self.goal.target_pose.pose.position.y=0.
			#self.goal.target_pose.pose.position.z=0.0
			#self.goal.target_pose.pose.orientation.w=1.0
			#self.goal.target_pose.header.frame_id = 'map'
        		#self.goal.target_pose.header.stamp = rospy.Time.now()
			#self.move_base.send_goal(self.goal)
			#state = self.move_base.get_state()
			#break
        	#else:
            rospy.loginfo("Waiting for BUD-E to reach the goal... " )#+ str(goal_states[state]))
	  #  call(["rostopic"] ["pub"] ["/move_base_simple/goal"] ["geometry_msgs/PoseStamped"] ["'{header: {stamp: now, frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'"]
	# TO-DO:
	# Write routines to take care of the case when the goal is aborted.l

     # POPend
          
        # Speak the recognized words in the selected voice
        #self.soundhandle.say(msg.data, self.voice)
        
        # Uncomment to play one of the built-in sounds
        #rospy.sleep(2)
        #self.soundhandle.play(5)
        
        # Uncomment to play a wave file
        #rospy.sleep(2)
        #self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")

    def cleanup(self):
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    rospy.init_node('talkback')
    try:
        TalkBack()
        rospy.spin()
    except:
        pass

