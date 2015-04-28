1. roslaunch BUDE.launch

2.roslaunch BUDE_Speech recognizer.launch

3.rosrun sound_play soundplay_node.py

4. rosrun BUDE_Speech talkback.py

5. [Scissor lift]
[Launch BUDE.launch before connecting Arduino to PC. Arduino is assumed ot be /dev/ttyACM1 and Hokuyo is assumed to be at /dev/ttyACM0]
rosrun rosserial_python serial_node.py /dev/ttyACM1

[!Set coordinates of kitchen and home for the current scan]

6. [Manipulator]
roslaunch bude_arm base.launch
roslaunch bude_arm meta.launch
rosrun bude_arm InverseKin.py

7. roslaunch freenect_launch freenect.launch (OR the Openni launch depending on what is installed)
8. rosrun detect_can detect_cylinder
9. rosrun detect_can parse_center


10.rosrun rosaria RosAria

BUD-E should say "Ready"

Say "Buddy" or say "Buddy get water" if you want it to execute the whole state machine we have so far

Say "Buddy coffee" to test the scissor lift alone.

