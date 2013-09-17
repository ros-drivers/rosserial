#include "ros/ros.h"
#include "std_msgs/String.h"

namespace rosserial {
#include "rosserial/ros.h"
#include "rosserial/std_msgs/String.h"
}
#include <gtest/gtest.h>

class ClientFixture : public ::testing::Test {
protected:
  virtual void SetUp() {
    
  }
  virtual void TearDown() {
    
  }

  rosserial::ros::NodeHandle client_nh; 
};

TEST_F(ClientFixture, basic_publish) {
  rosserial::std_msgs::String string;
  rosserial::ros::Publisher pub("chatter", &string);
  client_nh.advertise(pub);
  client_nh.initNode();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_publisher");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
