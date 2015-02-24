#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <stdlib.h>
#include <math.h>

float x;
float y;
float z;
int NUM_RUNS = 20;
int count = 0;
int reset_count = 0;
int RESET_MAX = 5;
float height_dev = 0.03;   // in meters
// The different states we can be in while parsing the stream of coordinates
// for the cluster center
enum coord_state {
	INIT,
	UPDATE,
	MONITOR
};
coord_state current_state = INIT;

ros::Publisher pub;

void parse_center(const geometry_msgs::Vector3 input)
{
std::cerr << current_state << " | " << input.x << " " << input.y << " " << input.z << std::endl;

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
std::cerr << "----------------" << std::endl;
std::cerr << x << " " << y << " " << z << std::endl;
std::cerr << "----------------" << std::endl;

				geometry_msgs::Vector3 output;
				output.x = x;
				output.y = y;
				output.z = z;
				pub.publish(output);

				count = 0;
				current_state = MONITOR;
			}
		break;
		// Monitor the stream of coordinates and if it deviates too much from
		// our current height, then we re-calculate the height
		case MONITOR:
			// Count how many times in a row our y displacement is greater
			// than 2.0 cm
			if (fabs(y-input.y) > height_dev) {
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

	pub = nh.advertise<geometry_msgs::Vector3>("bottle_center",100);
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
