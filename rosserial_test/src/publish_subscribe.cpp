#include "ros/ros.h"
#include "std_msgs/String.h"

namespace rosserial {
#include "rosserial/ros.h"
#include "rosserial/std_msgs/String.h"
}

#include <gtest/gtest.h>
#include "fixture.h"
#include "helpers.h"

TEST_F(ClientFixture, basic_publish) {
  // Rosserial client setup
  rosserial::std_msgs::String string_msg;
  rosserial::ros::Publisher client_pub("chatter", &string_msg);
  client_nh.advertise(client_pub);
  client_nh.initNode();
  char s[] = "Hello World";
  string_msg.data = s;
  
  // Roscpp verification setup
  StringCallback str_callback;
  ros::Subscriber check_sub = nh.subscribe("chatter", 1, &StringCallback::callback, &str_callback);

  for(int attempt = 0; attempt < 50; attempt++) { 
    client_pub.publish(&string_msg);
    client_nh.spinOnce();
    ros::spinOnce();
    if (str_callback.times_called > 0) break;
    ros::Duration(0.1).sleep();
  }
  EXPECT_GT(str_callback.times_called, 0);
  EXPECT_STREQ("Hello World", str_callback.last_msg.data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_publish_subscribe");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
