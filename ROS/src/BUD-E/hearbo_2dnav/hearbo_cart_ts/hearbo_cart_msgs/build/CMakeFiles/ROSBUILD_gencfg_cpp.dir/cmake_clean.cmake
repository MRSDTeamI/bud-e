FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hearbo_cart_msgs/msg"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/hearbo_cart_msgs/CartCommandConfig.h"
  "../docs/CartCommandConfig.dox"
  "../docs/CartCommandConfig-usage.dox"
  "../src/hearbo_cart_msgs/cfg/CartCommandConfig.py"
  "../docs/CartCommandConfig.wikidoc"
  "../cfg/cpp/hearbo_cart_msgs/TestConfig.h"
  "../docs/TestConfig.dox"
  "../docs/TestConfig-usage.dox"
  "../src/hearbo_cart_msgs/cfg/TestConfig.py"
  "../docs/TestConfig.wikidoc"
  "../cfg/cpp/hearbo_cart_msgs/CartVelocityConfig.h"
  "../docs/CartVelocityConfig.dox"
  "../docs/CartVelocityConfig-usage.dox"
  "../src/hearbo_cart_msgs/cfg/CartVelocityConfig.py"
  "../docs/CartVelocityConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
