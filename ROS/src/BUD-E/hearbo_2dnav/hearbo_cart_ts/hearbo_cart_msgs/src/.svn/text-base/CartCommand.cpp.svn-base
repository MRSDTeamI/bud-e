#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <hearbo_cart_msgs/CartCommandConfig.h>
#include <hearbo_cart_msgs/CartCommand.h>

ros::Publisher* g_pPublisher;

void callback(hearbo_cart_msgs::CartCommandConfig &config, uint32_t level)
{
  hearbo_cart_msgs::CartCommand oCartCommand;

  if(config.drive_flag)   oCartCommand.drive_flag   = oCartCommand.TRUE; else oCartCommand.drive_flag   = oCartCommand.FALSE;
  if(config.gain_flag)    oCartCommand.gain_flag    = oCartCommand.TRUE; else oCartCommand.gain_flag    = oCartCommand.FALSE;
  if(config.encoder_flag) oCartCommand.encoder_flag = oCartCommand.TRUE; else oCartCommand.encoder_flag = oCartCommand.FALSE;

  if(config.drive_mode == 0) oCartCommand.drive_mode = oCartCommand.STOP_MODE;
  if(config.drive_mode == 1) oCartCommand.drive_mode = oCartCommand.TURN_MODE;
  if(config.drive_mode == 2) oCartCommand.drive_mode = oCartCommand.DRIVE_MODE;
  if(config.drive_mode == 3) oCartCommand.drive_mode = oCartCommand.MANUAL_MODE;
  if(config.drive_mode == 4) oCartCommand.drive_mode = oCartCommand.RESTART_MODE;
  
  if(config.drive_id_0) oCartCommand.drive_id[0] = oCartCommand.TRUE; else oCartCommand.drive_id[0] = oCartCommand.FALSE;
  if(config.drive_id_1) oCartCommand.drive_id[1] = oCartCommand.TRUE; else oCartCommand.drive_id[1] = oCartCommand.FALSE;
  if(config.drive_id_2) oCartCommand.drive_id[2] = oCartCommand.TRUE; else oCartCommand.drive_id[2] = oCartCommand.FALSE;
  if(config.drive_id_3) oCartCommand.drive_id[3] = oCartCommand.TRUE; else oCartCommand.drive_id[3] = oCartCommand.FALSE;
  
  oCartCommand.each_vel[0] = (float)config.each_vel_0;
  oCartCommand.each_vel[1] = (float)config.each_vel_1;
  oCartCommand.each_vel[2] = (float)config.each_vel_2;
  oCartCommand.each_vel[3] = (float)config.each_vel_3;

  oCartCommand.each_ang[0] = (float)config.each_ang_0;
  oCartCommand.each_ang[1] = (float)config.each_ang_1;
  oCartCommand.each_ang[2] = (float)config.each_ang_2;
  oCartCommand.each_ang[3] = (float)config.each_ang_3;
  
  oCartCommand.all_vel = (float)config.all_vel;

  oCartCommand.all_ang = (float)config.all_ang;
  
  if(config.gain_id_ch0_0) oCartCommand.gain_id_ch0[0] = oCartCommand.TRUE; else oCartCommand.gain_id_ch0[0] = oCartCommand.FALSE;
  if(config.gain_id_ch0_1) oCartCommand.gain_id_ch0[1] = oCartCommand.TRUE; else oCartCommand.gain_id_ch0[1] = oCartCommand.FALSE;
  if(config.gain_id_ch0_2) oCartCommand.gain_id_ch0[2] = oCartCommand.TRUE; else oCartCommand.gain_id_ch0[2] = oCartCommand.FALSE;
  if(config.gain_id_ch0_3) oCartCommand.gain_id_ch0[3] = oCartCommand.TRUE; else oCartCommand.gain_id_ch0[3] = oCartCommand.FALSE;

  if(config.gain_id_ch1_0) oCartCommand.gain_id_ch1[0] = oCartCommand.TRUE; else oCartCommand.gain_id_ch1[0] = oCartCommand.FALSE;
  if(config.gain_id_ch1_1) oCartCommand.gain_id_ch1[1] = oCartCommand.TRUE; else oCartCommand.gain_id_ch1[1] = oCartCommand.FALSE;
  if(config.gain_id_ch1_2) oCartCommand.gain_id_ch1[2] = oCartCommand.TRUE; else oCartCommand.gain_id_ch1[2] = oCartCommand.FALSE;
  if(config.gain_id_ch1_3) oCartCommand.gain_id_ch1[3] = oCartCommand.TRUE; else oCartCommand.gain_id_ch1[3] = oCartCommand.FALSE;

  oCartCommand.gain_ch0[0] = (short)config.gain_ch0_0;
  oCartCommand.gain_ch0[1] = (short)config.gain_ch0_1;
  oCartCommand.gain_ch0[2] = (short)config.gain_ch0_2;
  oCartCommand.gain_ch0[3] = (short)config.gain_ch0_3;

  oCartCommand.gain_ch1[0] = (short)config.gain_ch1_0;
  oCartCommand.gain_ch1[1] = (short)config.gain_ch1_1;
  oCartCommand.gain_ch1[2] = (short)config.gain_ch1_2;
  oCartCommand.gain_ch1[3] = (short)config.gain_ch1_3;

  if(config.encoder_id_ch0_0) oCartCommand.encoder_id_ch0[0] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch0[0] = oCartCommand.FALSE;
  if(config.encoder_id_ch0_1) oCartCommand.encoder_id_ch0[1] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch0[1] = oCartCommand.FALSE;
  if(config.encoder_id_ch0_2) oCartCommand.encoder_id_ch0[2] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch0[2] = oCartCommand.FALSE;
  if(config.encoder_id_ch0_3) oCartCommand.encoder_id_ch0[3] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch0[3] = oCartCommand.FALSE;

  if(config.encoder_id_ch1_0) oCartCommand.encoder_id_ch1[0] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch1[0] = oCartCommand.FALSE;
  if(config.encoder_id_ch1_1) oCartCommand.encoder_id_ch1[1] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch1[1] = oCartCommand.FALSE;
  if(config.encoder_id_ch1_2) oCartCommand.encoder_id_ch1[2] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch1[2] = oCartCommand.FALSE;
  if(config.encoder_id_ch1_3) oCartCommand.encoder_id_ch1[3] = oCartCommand.TRUE; else oCartCommand.encoder_id_ch1[3] = oCartCommand.FALSE;

  oCartCommand.encoder_ch0[0] = (int)config.encoder_ch0_0;
  oCartCommand.encoder_ch0[1] = (int)config.encoder_ch0_1;
  oCartCommand.encoder_ch0[2] = (int)config.encoder_ch0_2;
  oCartCommand.encoder_ch0[3] = (int)config.encoder_ch0_3;

  oCartCommand.encoder_ch1[0] = (int)config.encoder_ch1_0;
  oCartCommand.encoder_ch1[1] = (int)config.encoder_ch1_1;
  oCartCommand.encoder_ch1[2] = (int)config.encoder_ch1_2;
  oCartCommand.encoder_ch1[3] = (int)config.encoder_ch1_3;

  if(config.publish_flag) g_pPublisher->publish( oCartCommand );

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CartCommandPublisherByDynReconf");
  ros::NodeHandle n;

  dynamic_reconfigure::Server<hearbo_cart_msgs::CartCommandConfig> srv;
  dynamic_reconfigure::Server<hearbo_cart_msgs::CartCommandConfig>::CallbackType f = boost::bind(&callback, _1, _2);

  ros::Publisher oPublisher = n.advertise<hearbo_cart_msgs::CartCommand>( "/hearbo_ixs_cart_ctrl/commands", 100 );
  g_pPublisher = &oPublisher;

  srv.setCallback(f);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
