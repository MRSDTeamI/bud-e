#include <dynamic_reconfigure/server.h>
#include <hearbo_cart_msgs/TestConfig.h>

void callback(hearbo_cart_msgs::TestConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i %i %f %s %i %i", config.int_enum_, config.int_, config.double_, config.str_.c_str(), (int) config.bool_, config.level);
  
  config.int_ |= 1;
  config.double_ = -config.double_;
  config.str_ += "A";
  config.bool_ = !config.bool_;
  config.level = level;

  ROS_INFO("Reconfigured to     : %i %i %f %s %i %i", config.int_enum_, config.int_, config.double_, config.str_.c_str(), (int) config.bool_, config.level);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_reconfigure_test_server");
  dynamic_reconfigure::Server<hearbo_cart_msgs::TestConfig> srv;
  dynamic_reconfigure::Server<hearbo_cart_msgs::TestConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);
  ROS_INFO("Constants are: %i %f %s %i", hearbo_cart_msgs::Test_int_const, hearbo_cart_msgs::Test_double_const, hearbo_cart_msgs::Test_str_const, (int) hearbo_cart_msgs::Test_bool_const);
  ROS_INFO("Starting to spin...");
  ros::spin();
  return 0;
}
