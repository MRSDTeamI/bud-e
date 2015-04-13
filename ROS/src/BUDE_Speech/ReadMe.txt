1. roslaunch BUDE.launch

2.roslaunch BUDE_Speech recognizer.launch

3.rosrun sound_play soundplay_node.py

4. rosrun BUDE_Speech talkback.py

[Scissor lift]
[Launch BUDE.launch before connecting Arduino to PC. Arduino is assumed ot be /dev/ttyACM1 and Hokuyo is assumed to be at /dev/ttyACM0]
rosrun rosserial_python serial_node.py /dev/ttyACM1

[!Set coordinates of kitchen and home for the current scan]

5.rosrun rosaria RosAria

6. roslaunch freenect_launch freenect.launch
7. rosrun detect_can detect_cylinder
8. rosrun detect_can parse_center

BUD-E should say "Ready"

Say "Buddy" or say "Buddy get water" if you want it to execute the whole state machine we have so far

Say "Buddy coffee" to test the scissor lift alone.

