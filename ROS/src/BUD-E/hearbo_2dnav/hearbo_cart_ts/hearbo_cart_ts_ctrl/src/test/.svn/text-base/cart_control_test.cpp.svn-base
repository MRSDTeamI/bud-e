#include "ros/ros.h"
#include "hearbo_cart_msgs/CartVelocity.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_control");
  ros::NodeHandle n;
  
  ros::Publisher oCartVelocityPublisher = n.advertise<hearbo_cart_msgs::CartVelocity>("/hearbo_ts_cart_ctrl/commands", 1);
  
  char pcEnteredText[32];
  while (ros::ok())
  {
    printf("Send another velocity command? ");
    std::cin.getline( pcEnteredText, 32 );
  
    if( strcmp(pcEnteredText, "n") == 0 || strcmp(pcEnteredText, "N") == 0)
      break;
    else if( strcmp(pcEnteredText, "y") != 0 && strcmp(pcEnteredText, "Y") != 0)
    {
      printf("Illegal reply: %s\n", pcEnteredText);
      continue;
    }
    
    hearbo_cart_msgs::CartVelocity msg;
    float fTemp;
    
    printf("Enter X velocity: ");
    std::cin.getline( pcEnteredText, 32 );
    sscanf(pcEnteredText, "%f", &fTemp);
    msg.velocity_x = fTemp;
    
    printf("Enter Y velocity: ");
    std::cin.getline( pcEnteredText, 32 );
    sscanf(pcEnteredText, "%f", &fTemp);
    msg.velocity_y = fTemp;
    
    printf("Enter rot velocity: ");
    std::cin.getline( pcEnteredText, 32 );
    sscanf(pcEnteredText, "%f", &fTemp);
    msg.velocity_rot = fTemp;

    ROS_INFO("Command : fVelx : %f, fVely : %f, fVelr : %f",msg.velocity_x,msg.velocity_y,msg.velocity_rot);
    
    oCartVelocityPublisher.publish( msg );
    
    ros::spinOnce();
  }
  
  return 0;
}





