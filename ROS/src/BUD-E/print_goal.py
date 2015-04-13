#/usr/bin/python

import sys
import re

# Amount of time to wait at velocity == 0 to declare goal reached
sec_to_goal = 2

last_zero = False
zero_time = 0

while True:
	# [ INFO] [1416787190.800175371]: Command received: Linear.x=[0.000000]  Angular.z=[0.000000]
	# NEW FORMAT:
	# [ INFO] [1416783081.527455584]: new speed: [0.00,-0.40](1416783081.527)
	
	line = sys.stdin.readline()
	if "new speed" in line:
		my_arr = line.split(":")
		m = re.search('\[\s+INFO\].*\[(\d+\.\d+)\]', my_arr[0])
		if m:
			timestamp = m.group(1)
		else:
			print "Unable to process timestamp " + my_arr[0]
			sys.exit(1)

		m = re.search('\[(\d+\.\d+),\-?(\d+\.\d+)\]', my_arr[2])
		if m:
			lin_vel = m.group(1)
			ang_vel = m.group(2)
		else:
			print "Unable to process velocities" + my_arr[2]
			sys.exit(1)

		#print timestamp
		#print lin_vel
		#print ang_vel

		if (float(lin_vel) == 0) and (float(ang_vel) == 0):
			if last_zero:
				if float(timestamp) - zero_time > sec_to_goal:
					print "GOAL REACHED at ", float(timestamp)
					sys.exit(0)
			else:
				last_zero = True
				zero_time = float(timestamp)
		else:
			last_zero = False

	#sys.stdout.write(line)
	sys.stdout.flush()
