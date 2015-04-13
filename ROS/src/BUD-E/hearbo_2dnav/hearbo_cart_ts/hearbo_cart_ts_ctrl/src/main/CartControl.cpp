// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include "geometry_msgs/Twist.h"			//type of msgs for move_base

// Cart
#include "CartCommandMaker.h"

CartCommandMaker g_oCommandMaker;

void setCartVelocity(const geometry_msgs::Twist::ConstPtr& msg);

int main(int argc, char **argv)
{
  bool bDoConnect = true;
  
  // Connect to robot
  bool bConnected = g_oCommandMaker.initialize(bDoConnect);
  if( !bConnected ) ROS_WARN( "Could not connect to cart." );
  if( !bDoConnect )

  {
    ROS_WARN( "Not connecting to cart (change flag bDoConnect)." );
    bConnected = false;
  }
  
  // Initialize ROS node
  ros::init(argc, argv, "cart_control");
  ros::NodeHandle n;
      
  // Activate the cart
  if( bConnected )
  {
    g_oCommandMaker.sendActivationCommand();
    ROS_INFO("Cart activated."); 
  }
  
  // Subscribe to all command messages
  ros::Subscriber sub0 = n.subscribe("cmd_vel", 1, setCartVelocity);
  ROS_INFO("Ready for ROS commands.hi..!!");
  
  ros::spin();

  g_oCommandMaker.terminate();
  return 0;
}

void setCartVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{

  // ROS_INFO("Received : x : %f, y : %f, r : %f",msg->velocity_x,msg->velocity_y,msg->velocity_rot);

  if( fabs(msg->angular.z*95.0) < 15.1000 ) // if the rotational velocity is zero
  {
    // if the translational velocity is zero too, stop the robot
    if( (fabs(msg->linear.y)*1000.0 < 0.0001) && (fabs(msg->linear.x)*1000.0 < 0.0001) )
    {
      g_oCommandMaker.sendRotationalVelocityCommand( 0 );
      g_oCommandMaker.sendTranslationalVelocityCommand( 0, 0 );
    }
    // send a translational velocity command
    else
    {
      g_oCommandMaker.sendTranslationalVelocityCommand( msg->linear.y*1000.0, msg->linear.x*1000.0 );
    }
  }
  else // if the rotational velocity is not zero
  {
    // send a rotational velocity command
    g_oCommandMaker.sendRotationalVelocityCommand( msg->angular.z*95.0 );
  }
}




