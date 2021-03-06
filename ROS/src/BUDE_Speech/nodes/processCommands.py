#!/usr/bin/env python


import roslib; roslib.load_manifest('BUDE_Speech')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

class voice_cmd_vel:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        self.paused = False
        
        # Initialize the Twist message that will be publish.
        self.msg = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,1)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speechCb)
        # Add how many very keyword command pair as you can. But BUD-E will respond to only a subset of these for safety.
        # A mapping from keywords to commands.
        self.keywords_to_command = {'hello': ['hey buddy', 'hello buddy', 'hi buddy','buddy'],
                                    'kitchen': ['buddy go to the kitchen','buddy kitchen','kitchen','go to kitchen','buddy go to kitchen'],
				    'water':['buddy fetch water','buddy fetch water water','buddy fetch water bottle','buddy bring water','buddy bring water bottle'],
				    'kitchenWater':['buddy get water from kitchen','buddy get water bottle from the kitchen','buddy get water water from the kitchen','buddy fetch water from kitchen'],
				    'pizza':['get me pizza','buddy get pizza'],
				    'coffee':['can you get me coffe','can you bring coffe']
                                    } 
        
        #rospy.loginfo("Ready to receive voice commands")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.msg)
            r.sleep()                       
            
    def get_command(self, data):
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speechCb(self, msg):        
        command = self.get_command(msg.data)
        
       # rospy.loginfo("Command: " + str(command))
        
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
            
        if self.paused:
            return       
        
#########
        if command == 'hello':    
            rospy.loginfo("Hi! How can I help you?")
        if command=='go':
 	    rospy.loginfo("Where should I go?")
	if command=='kitchen':
 	    rospy.loginfo("Okay. I know I have to go to the kitchen BUT, I don't know yet where the kitchen is!")
	if command=='water':
 	    rospy.loginfo("I am going to the fountain of youth to get you water.")
	if command=='kitchenWater':
 	    rospy.loginfo("Got you! I am going to the kitchen to fetch you water (If only I had legs).")
	if command=='pizza':
 	    rospy.loginfo("Hagen/ Julie has arranged for it this Friday after the seminar")
	if command=='coffee':
 	    rospy.loginfo("I am sorry. I cannot climb stairs. You can get a cup of coffe from NSH 3rd or 4th floor.")
	if command=='none':
 	    rospy.loginfo("I am sorry. I did not understand what you said. Can you please say it again?")
############
        if command == 'forward':    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
            
        elif command == 'rotate left':
            self.msg.linear.x = 0
            self.msg.angular.z = self.angular_speed
                
        elif command == 'rotate right':  
            self.msg.linear.x = 0      
            self.msg.angular.z = -self.angular_speed
            
        elif command == 'turn left':
            if self.msg.linear.x != 0:
                self.msg.angular.z += self.angular_increment
            else:        
                self.msg.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.msg.linear.x != 0:
                self.msg.angular.z -= self.angular_increment
            else:        
                self.msg.angular.z = -self.angular_speed
                
        elif command == 'backward':
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.msg = Twist()
        
        elif command == 'faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.msg.linear.x != 0:
                self.msg.linear.x += copysign(self.linear_increment, self.msg.linear.x)
            if self.msg.angular.z != 0:
                self.msg.angular.z += copysign(self.angular_increment, self.msg.angular.z)
            
        elif command == 'slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.msg.linear.x != 0:
                self.msg.linear.x -= copysign(self.linear_increment, self.msg.linear.x)
            if self.msg.angular.z != 0:
                self.msg.angular.z -= copysign(self.angular_increment, self.msg.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)
        
            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
            
            if self.msg.linear.x != 0:
                self.msg.linear.x = copysign(self.speed, self.msg.linear.x)

            if self.msg.angular.z != 0:
                self.msg.angular.z = copysign(self.angular_speed, self.msg.angular.z)
                
        else:
            return

        self.msg.linear.x = min(self.max_speed, max(-self.max_speed, self.msg.linear.x))
        self.msg.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.msg.angular.z))

    def cleanup(self):
        # When shutting down be sure to stop the robot!  Publish a Twist message consisting of all zeros.
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_nav')
    try:
        voice_cmd_vel()
    except:
        pass

