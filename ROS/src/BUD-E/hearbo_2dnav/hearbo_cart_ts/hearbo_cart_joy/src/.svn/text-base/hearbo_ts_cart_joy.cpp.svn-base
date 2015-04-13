// Manual Settings
#define MAX_VELOCITY_DRIVE_MODE 10.0
#define MAX_VELOCITY_TURN_MODE 10.0

#include <ros/ros.h>
#include <hearbo_cart_msgs/CartVelocity.h>
#include <joystick_hri/Joy.h>
hearbo_cart_msgs::CartVelocity msg;

class TeleopCart
{
public:
  TeleopCart();

private:
  void joyCallback(const joystick_hri::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
  double max_vel_d_;
  double max_vel_t_;
  ros::Publisher cart_pub_;
  ros::Subscriber joy_sub_;
};

TeleopCart::TeleopCart():
  max_vel_d_(MAX_VELOCITY_DRIVE_MODE),
  max_vel_t_(MAX_VELOCITY_TURN_MODE)
{
  nh_.param("max_vel_drive_mode", max_vel_d_, max_vel_d_);
  nh_.param("max_vel_turn_mode", max_vel_t_, max_vel_t_);
  cart_pub_ = nh_.advertise<hearbo_cart_msgs::CartVelocity>("/hearbo_ts_cart_ctrl/commands", 1);
  joy_sub_ = nh_.subscribe<joystick_hri::Joy>("joy", 10, &TeleopCart::joyCallback, this);
}

void TeleopCart::joyCallback(const joystick_hri::Joy::ConstPtr& joy)
{
  if(joy->buttons[0] == 1){
    ROS_INFO("DRIVE : FORWARD");
    msg.velocity_x   = 0.0;
    msg.velocity_y   = -1.0 * (float)max_vel_d_;
    msg.velocity_rot = 0.0;
  }
  else if(joy->buttons[2] == 1){
    ROS_INFO("DRIVE : BACKWARD");
    msg.velocity_x   = 0.0;
    msg.velocity_y   = (float)max_vel_d_;
    msg.velocity_rot = 0.0;
  }
  else if(joy->buttons[1] == 1){
    ROS_INFO("TURN : CLOCKWISE");
    msg.velocity_x   = 0.0;
    msg.velocity_y   = 0.0;
    msg.velocity_rot = -1.0 * (float)max_vel_t_;
  }
  else if(joy->buttons[3] == 1){
    ROS_INFO("TURN : ANTICLOCKWISE");
    msg.velocity_x   = 0.0;
    msg.velocity_y   = 0.0;
    msg.velocity_rot = (float)max_vel_t_;
  }
  else{
    ROS_INFO("STOP");
    msg.velocity_x   = 0.0;
    msg.velocity_y   = 0.0;
    msg.velocity_rot = 0.0;
  }
  cart_pub_.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hearbo_ts_cart_joy");
  TeleopCart teleop_cart;
  ros::spin();
}
