// Manual Settings
#define MAX_VELOCITY_DRIVE_MODE 50.0
#define MAX_VELOCITY_TURN_MODE 50.0

#include <ros/ros.h>
#include <hearbo_cart_msgs/CartCommand.h>
#include <joystick_hri/Joy.h>
hearbo_cart_msgs::CartCommand msg;

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
  cart_pub_ = nh_.advertise<hearbo_cart_msgs::CartCommand>("/hearbo_ixs_cart_ctrl/commands", 1);
  joy_sub_ = nh_.subscribe<joystick_hri::Joy>("joy", 10, &TeleopCart::joyCallback, this);
}

void TeleopCart::joyCallback(const joystick_hri::Joy::ConstPtr& joy)
{
  if(joy->buttons[0] == 1){
    ROS_INFO("DRIVE : FORWARD");
    msg.drive_flag = msg.TRUE;
    msg.drive_mode = msg.DRIVE_MODE;
    msg.drive_id[0] = msg.TRUE;
    msg.drive_id[1] = msg.TRUE;
    msg.drive_id[2] = msg.TRUE;
    msg.drive_id[3] = msg.TRUE;
    msg.all_vel = (float)max_vel_d_;
    msg.all_ang = 0.0;
  }
  else if(joy->buttons[2] == 1){
    ROS_INFO("DRIVE : BACKWARD");
    msg.drive_flag = msg.TRUE;
    msg.drive_mode = msg.DRIVE_MODE;
    msg.drive_id[0] = msg.TRUE;
    msg.drive_id[1] = msg.TRUE;
    msg.drive_id[2] = msg.TRUE;
    msg.drive_id[3] = msg.TRUE;
    msg.all_vel = -1.0 * (float)max_vel_d_;
    msg.all_ang = 0.0;
  }
  else if(joy->buttons[1] == 1){
    ROS_INFO("TURN : CLOCKWISE");
    msg.drive_flag = msg.TRUE;
    msg.drive_mode = msg.TURN_MODE;
    msg.drive_id[0] = msg.TRUE;
    msg.drive_id[1] = msg.TRUE;
    msg.drive_id[2] = msg.TRUE;
    msg.drive_id[3] = msg.TRUE;
    msg.all_vel = (float)max_vel_t_;
    msg.all_ang = 0.0;
  }
  else if(joy->buttons[3] == 1){
    ROS_INFO("TURN : ANTICLOCKWISE");
    msg.drive_flag = msg.TRUE;
    msg.drive_mode = msg.TURN_MODE;
    msg.drive_id[0] = msg.TRUE;
    msg.drive_id[1] = msg.TRUE;
    msg.drive_id[2] = msg.TRUE;
    msg.drive_id[3] = msg.TRUE;
    msg.all_vel = -1.0 * (float)max_vel_t_;
    msg.all_ang = 0.0;
  }
  else{
    ROS_INFO("STOP");
    if(msg.drive_mode == msg.STOP_MODE) msg.drive_mode = msg.DRIVE_MODE;
    msg.drive_flag = msg.TRUE;
    msg.drive_id[0] = msg.TRUE;
    msg.drive_id[1] = msg.TRUE;
    msg.drive_id[2] = msg.TRUE;
    msg.drive_id[3] = msg.TRUE;
    msg.all_vel = 0.0;
  }
  cart_pub_.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hearbo_ixs_cart_joy");
  TeleopCart teleop_cart;
  ros::spin();
}
