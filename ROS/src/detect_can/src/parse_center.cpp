#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

float x;
float y;
float z;
int NUM_RUNS = 20;
int count = 0;
int reset_count = 0;
int RESET_MAX = 10;
float height_dev = 0.05;   // in meters
// The different states we can be in while parsing the stream of coordinates
// for the cluster center
enum coord_state {
	INIT,
	UPDATE,
	MONITOR
};
coord_state current_state = INIT;
bool pause_parser = false;

ros::Publisher pub_bot;
ros::Publisher pub_sl;
ros::Publisher pub_vision;

void reset_state(const std_msgs::Bool input)
{
    current_state = INIT;
    count = 0;
    if (input.data) {
        pause_parser = false;
    } else {
        pause_parser = true;
    }
}

void parse_center(const geometry_msgs::Vector3 input)
{
    if (pause_parser)
        return;

    std::out << current_state << " | " << input.x << " " << input.y << " " << input.z << std::endl;

	switch(current_state) {
		// Just take the first input and mark it as initial point
		case INIT:
			x = input.x;
			y = input.y;
			z = input.z;
			current_state = UPDATE;
		break;
		// Average the points over the next NUM_RUNS
		case UPDATE:
			if (count < NUM_RUNS) {
				x = (x + input.x)/2;
				y = (y + input.y)/2;
				z = (z + input.z)/2;

            	count++;
			} else {
				// Print our coordinate
                std::cout << "----------------" << std::endl;
                std::cout << x << " " << y << " " << z << std::endl;
                std::cout << "----------------" << std::endl;

				geometry_msgs::Vector3 arm_output;
				/******
				Convert coordinates to arm coordinates.
				Kinect coordinates (Kinect frame):
					x to the left
					y going down
					z going forward
				Arm coordinates (Arm frame):
					x going forward
					y to the left
					z going up

				Furthermore, because the Kinect is mounted above and behind
				the arm, we need to subtract from the height and depth
				measurements.
				
				
				******/
				//arm_output.x = x * 100;
				//arm_output.y = y * 100;
				//arm_output.z = z * 100;
				
				double kinect_depth = 0.26; // Kinect is behind the arm.
				double kinect_height = 0.20; // Kinect is above the arm.
				double bottle_ht = 0.088;   // approx. bottle center height (above gnd plane)
                double kinect_x_coeff = 0; // For some reason, it keeps reporting objects a little bit offset.
				
				arm_output.x = z - kinect_depth - 0.02;
				arm_output.y = -1*(x - kinect_x_coeff) + 0.09;
				arm_output.z = -1*(y - kinect_height) + 0.08;
				pub_bot.publish(arm_output);

				// for scissor lift
				// range of height for PR 9 should be 0-20cm
				// cap on these values only for this PR
                arm_output.z = arm_output.z - bottle_ht;
				if (arm_output.z < 0)  
					arm_output.z = 0;
				if (arm_output.z > 0.1)
					arm_output.z = 0.1;
				std_msgs::Int32 sl_output;
				sl_output.data = arm_output.z*100;
				pub_sl.publish(sl_output);

				count = 0;
				//current_state = MONITOR;

                // tell vision to stop, arm needs to turn it back on once it is done.
                std_msgs::Bool run_vision;
                run_vision.data = false;
                pub_vision.publish(run_vision);
				current_state = INIT;
			}
		break;
		// Monitor the stream of coordinates and if it deviates too much from
		// our current height, then we re-calculate the height
		case MONITOR:
			// Count how many times in a row our y displacement is greater
			// than 2.0 cm
			if ((fabs(y-input.y) > height_dev) || (fabs(x-input.x) > height_dev)
                || (fabs(z-input.z) > height_dev)) {
				reset_count++;
			} else {
				reset_count = 0;
			}
			// if more than 5 times in a row then we re-init coord values
			if (reset_count == RESET_MAX) {
				reset_count = 0;
				current_state = INIT;
			}
		break;
		default:
			// do nothing
			std::cerr << "Should never get here" << std::endl;
	}
}

int
main (int argc, char** argv)
{
	ros::init (argc, argv, "parse_center");
	ros::NodeHandle nh;
 
	ros::Subscriber sub = nh.subscribe ("cluster_center", 100, parse_center);
    ros::Subscriber reset = nh.subscribe ("reset_parse", 1, reset_state);

	// bottle coordinates for arm
	pub_bot = nh.advertise<geometry_msgs::Vector3>("bottle_center",100);

	// height (0-20cm) for scissor lift
	pub_sl = nh.advertise<std_msgs::Int32>("target_height",100);

    // To pause the vision module once we've given the bottle coordinate.
    // This is so the arm can try to grasp without vision falsely picking
    // up random objects.
    pub_vision = nh.advertise<std_msgs::Bool>("start_vision", 10);
//
//	// Sets up the random number generator
//	srand(time(0));
//	// Sets the loop to publish at rate of 10Hz
//	ros::Rate rate(1);
//
//	while (ros::ok()) {
//		// Declares the message to be sent
//		geometry_msgs::Vector3 can_coord;
//
//		// Random x value between -2 and 2
//		can_coord.x = 4 * double(rand())/double(RAND_MAX)-2;
//		// Random y value between -2 and 2
//		can_coord.y = 4 * double(rand())/double(RAND_MAX)-2;
//		// Random z value between -2 and 2
//		can_coord.z = 4 * double(rand())/double(RAND_MAX)-2;
//		
//		pub.publish(can_coord);
//
//		rate.sleep();
//	}

	ros::spin();
}
