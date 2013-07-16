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
  ros::AsyncSpinner ros_spinner(1);
  ros_spinner.start();

  // Monitor ROS for shutdown, and stop the io_service accordingly.
  AsyncOkPoll ok_poll(io_service, boost::posix_time::milliseconds(500), ros::ok);

  // Begin rosserial session with serial port. 
  SerialSession s(io_service);

  // Set up serial port specifics.
  boost::system::error_code ec;
  s.socket().open("/dev/ttyUSB0", ec);
  if (ec) {
    // Todo: repeated reattempts.
    ROS_FATAL("Unable to open port.");
    return 1;
  }
  s.socket().set_option(boost::asio::serial_port_base::baud_rate(57600));
  s.socket().set_option(boost::asio::serial_port_base::character_size(8));
  s.socket().set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  s.socket().set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  s.socket().set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  // Kick off the session.
  io_service.run();

  return 0;
}
