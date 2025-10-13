#include "ros/ros.h"
#include "std_srvs/Empty.h"

namespace rosserial {
#include "rosserial_test/ros.h"
#include "rosserial/std_srvs/Empty.h"
}

#include <gtest/gtest.h>
#include "rosserial_test/fixture.h"
#include <boost/thread.hpp>

/* TODO */
/* this test is imcomplete, since the response is not checked. */

bool receive = false;
static void callback(const rosserial::std_srvs::Empty::Request & req, rosserial::std_srvs::Empty::Response & res){
  receive = true;
}

/**
 * Single service call from a rosserial-connected client,
 * verified from a roscpp rosservice server.
 */
TEST_F(SingleClientFixture, single_service_server) {

  rosserial::ros::ServiceServer<rosserial::std_srvs::Empty::Request, rosserial::std_srvs::Empty::Response> client_server("test_srv",&callback);
  client_nh.advertiseService(client_server);
  client_nh.initNode();

  // Roscpp rosservice client to call for the client.
  ros::ServiceClient test_client = nh.serviceClient<std_srvs::Empty>("test_srv");
  std_srvs::Empty srv;

  for(int attempt = 0; attempt < 50; attempt++) {
    if (attempt % 10 == 0 && attempt > 0) {

      if (test_client.call(srv)) {
        ROS_ERROR("Receive response");
        break;
      }
      else {
        ROS_ERROR("Failed to call service test_srv");
      }
    }

    if(receive) break;
    client_nh.spinOnce();
    ros::Duration(0.1).sleep();
  }

  EXPECT_TRUE(receive);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_service_server");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
