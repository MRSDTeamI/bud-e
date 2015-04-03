#!/usr/bin/env python
from quickui.QuickUi import *

m = gui("BUD-E arm joint control","vertical",
        group("Joints","vertical",
              

		ros_slider("shoulder_pan_joint","/shoulder_pan_joint/command","std_msgs/Float64",".data",(-1.8,1.8),0.0),

		ros_slider("shoulder_pitch_joint","/shoulder_pitch_joint/command","std_msgs/Float64",".data",(-1.8,1.8),0.0),

		ros_slider("elbow_flex_joint","/elbow_flex_joint/command","std_msgs/Float64",".data",(-1.8,1.8),0.0),

		ros_slider("wrist_roll_joint","/wrist_roll_joint/command","std_msgs/Float64",".data",(-1.8,1.8),0.0)
#)
),

		ros_slider("Gripper","/gripper/command","std_msgs/Float64",".data",(-0.5,1.1),0.0)
        )

run(m,debug=True)


#      - shoulder_pan_joint
#      - shoulder_pitch_joint
#      - elbow_flex_joint
#      - wrist_roll_joint
