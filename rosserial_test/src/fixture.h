
#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h> 

class ClientFixture : public ::testing::Test {
protected:
  virtual void SetUp() {
    // TODO: Check an arg or parameter for whether we should create a serial
    // or a socket loopback. For now, the socket logic is just included here.

    rosserial::ClientComms::fd = socket(AF_INET, SOCK_STREAM, 0); 
    ASSERT_GE(rosserial::ClientComms::fd, 0);
    fcntl(rosserial::ClientComms::fd, F_SETFL, O_NONBLOCK);

    memset(&serv_addr, '0', sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(11411);
    ASSERT_GE(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr), 0);

    // Try a bunch of times; we don't know how long it will take for the
    // server to come up.
    for (int attempt = 0; attempt < 10; attempt++) {
      if (connect(rosserial::ClientComms::fd, 
          (struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0) {
        // Connection successful.
        return;  
      }
      ros::Duration(0.5).sleep();
    } 
    ASSERT_TRUE(false) << "Unable to connect to roserial server.";
  }
  virtual void TearDown() {
    close(rosserial::ClientComms::fd);
  }

  rosserial::ros::NodeHandle client_nh; 
  ros::NodeHandle nh;
  struct sockaddr_in serv_addr; 
};


