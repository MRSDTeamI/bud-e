#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include "geometry_msgs/Twist.h"			//type of msgs for move_base
#include "hearbo_cart_msgs/CartVelocity.h"		//type of msgs for hearbo

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
/*
    cout << "velocity_x=" << cmd_vel->linear.x << endl;
    cout << "velocity_y=" << cmd_vel->linear.y << endl;
    cout << "velocity_rot=" << cmd_vel->angular.z << endl;
*/
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "link_node");   
  ros::NodeHandle n;   
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_velCallback);
  ros::Publisher chatter_pub = n.advertise<hearbo_cart_msgs::CartVelocity>("hearbo_ts_cart_ctrl/commands", 1000);

  ros::Rate loop_rate(10);   
/*
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);

    ros::spinOnce();

    
  } 
*/

  ros::spin();

  return 0;
}
