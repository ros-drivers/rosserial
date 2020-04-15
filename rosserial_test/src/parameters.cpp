#include "ros/ros.h"
#include "std_msgs/String.h"

namespace rosserial {
#include "rosserial_test/ros.h"
}

#include <gtest/gtest.h>
#include "rosserial_test/fixture.h"
#include "rosserial_test/helpers.h"

TEST_F(SingleClientFixture, single_bool_param) {
  client_nh.initNode();
  bool value = false;
  client_nh.getParam("bool_value", &value);
  EXPECT_EQ(value, true);
}

TEST_F(SingleClientFixture, single_int_param) {
  client_nh.initNode();
  int value = 0;
  client_nh.getParam("int_value", &value);
  EXPECT_EQ(value, 1);
}

TEST_F(SingleClientFixture, single_float_param) {
  client_nh.initNode();
  float value = 0.f;
  client_nh.getParam("float_value", &value);
  EXPECT_EQ(value, 0.5f);
}

TEST_F(SingleClientFixture, single_string_param) {
  client_nh.initNode();
  char textbuf[20] = "";
  char* buf_ptr = textbuf;
  client_nh.getParam("string_value", &buf_ptr);
  EXPECT_STREQ(textbuf, "sample text");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_publish_subscribe");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
