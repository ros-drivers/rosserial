
#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h> 

#include "ros/ros.h"

namespace rosserial {
#include "rosserial/ros.h"
}

class AbstractSetup {
public:
  virtual void SetUp()=0;
  int fd;
};

class SerialSetup : public AbstractSetup {
public:
  virtual void SetUp() {
  } 
};

class SocketSetup : public AbstractSetup {
public:
  virtual void SetUp() {
    fd = socket(AF_INET, SOCK_STREAM, 0); 
    ASSERT_GE(fd, 0);
    fcntl(fd, F_SETFL, O_NONBLOCK);

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(11411);
    ASSERT_GE(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr), 0);

    // Try a bunch of times; we don't know how long it will take for the
    // server to come up.
    for (int attempt = 0; attempt < 10; attempt++) {
      if (connect(fd, 
          (struct sockaddr *)&serv_addr, 
          sizeof(serv_addr)) >= 0) {
        // Connection successful.
        return;  
      }
      ros::Duration(0.5).sleep();
    } 
    FAIL() << "Unable to connect to rosserial socket server.";
  }
  struct sockaddr_in serv_addr; 
};

class ClientFixture : public ::testing::Test {
public:
  static void SetMode(std::string& mode) {
    ROS_INFO_STREAM("Using test mode [" << mode << "]");
    if (mode == "socket") {
      setup = new SocketSetup();
    } else if (mode == "serial") {
      setup = new SerialSetup();
    } else {
      FAIL() << "Mode specified other than 'serial' or 'socket'.";
    }
  }

protected:
  virtual void SetUp() {
    ASSERT_TRUE(setup != NULL) << "ClientFixture's setup helper pointer is null. Be sure to invoke the tests with the 'socket' or 'serial' arg.";
    setup->SetUp();
    rosserial::ClientComms::fd = setup->fd;
  }
  virtual void TearDown() {
    close(rosserial::ClientComms::fd);
  }

  rosserial::ros::NodeHandle client_nh; 
  ros::NodeHandle nh;
  static AbstractSetup* setup;
};
AbstractSetup* ClientFixture::setup = NULL;


