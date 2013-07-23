#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "Session.h"
#include "AsyncOkPoll.h"


typedef Session<boost::asio::serial_port> SerialSession;


int main(int argc, char* argv[])
{
  boost::asio::io_service io_service;

  // Initialize ROS.
  ros::init(argc, argv, "rosserial_server_serial_node");
  ros::NodeHandle nh("~");
  std::string port; nh.param<std::string>("port", port, "/dev/ttyACM0");
  int baud; nh.param<int>("baud", baud, 57600);

  // ROS background thread.
  ros::AsyncSpinner ros_spinner(1);
  ros_spinner.start();

  // Monitor ROS for shutdown, and stop the io_service accordingly.
  AsyncOkPoll ok_poll(io_service, boost::posix_time::milliseconds(500), ros::ok);

  // Begin rosserial session with serial port. 
  SerialSession s(io_service);

  // Set up serial port specifics.
  ROS_INFO("Opening serial port.");
  boost::system::error_code ec;
  s.socket().open(port, ec);
  if (ec) {
    // Todo: repeated reattempts.
    ROS_FATAL_STREAM("Unable to open port " << port);
    return 1;
  }

  typedef boost::asio::serial_port_base serial;
  s.socket().set_option(serial::baud_rate(baud));
  s.socket().set_option(serial::character_size(8));
  s.socket().set_option(serial::stop_bits(serial::stop_bits::one));
  s.socket().set_option(serial::parity(serial::parity::none));
  s.socket().set_option(serial::flow_control(serial::flow_control::none));

  // Kick off the session.
  s.start();
  io_service.run();

  return 0;
}
