#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <hearbo_cart_msgs/CartVelocityConfig.h>
#include <hearbo_cart_msgs/CartVelocity.h>

ros::Publisher* g_pPublisher;

void callback(hearbo_cart_msgs::CartVelocityConfig &config, uint32_t level)
{
  hearbo_cart_msgs::CartVelocity oCartVelocity;
  
  oCartVelocity.velocity_x   = (float)config.velocity_x;
  oCartVelocity.velocity_y   = (float)config.velocity_y;
  oCartVelocity.velocity_rot = (float)config.velocity_rot;

  if(config.stop){
    oCartVelocity.velocity_x   = (float)0.0;
    oCartVelocity.velocity_y   = (float)0.0;
    oCartVelocity.velocity_rot = (float)0.0;
    config.stop = false;
  }

  g_pPublisher->publish( oCartVelocity );

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CartVelocityPublisherByDynReconf");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<hearbo_cart_msgs::CartVelocityConfig> srv;
  dynamic_reconfigure::Server<hearbo_cart_msgs::CartVelocityConfig>::CallbackType f = boost::bind(&callback, _1, _2);

  ros::Publisher oPublisher = n.advertise<hearbo_cart_msgs::CartVelocity>( "/hearbo_ts_cart_ctrl/commands", 100 );
  g_pPublisher = &oPublisher;

  srv.setCallback(f);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
