/**
 *
 *  \file
 *  \brief      Main entry point for the serial node.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to code@clearpathrobotics.com 
 *
 */

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "Session.h"
#include "AsyncOkPoll.h"


class SerialSession : public Session<boost::asio::serial_port>
{
public:
  SerialSession(boost::asio::io_service& io_service, std::string port, int baud)
    : Session(io_service), port_(port), baud_(baud), // io_service_(io_service),
      timer_(io_service), interval_(boost::posix_time::milliseconds(1000))
  {
    connect_with_reconnection();
  }

private:
  ~SerialSession() {
    if (ros::ok()) {
      new SerialSession(socket().get_io_service(), port_, baud_);
    }
  }

  bool attempt_connection(bool log_errors = true)
  {
    if (log_errors) ROS_INFO("Opening serial port.");
    boost::system::error_code ec;
    socket().open(port_, ec);
    if (ec) {
      if (log_errors) ROS_ERROR_STREAM("Unable to open port " << port_);
      return false;
    }

    typedef boost::asio::serial_port_base serial;
    socket().set_option(serial::baud_rate(baud_));
    socket().set_option(serial::character_size(8));
    socket().set_option(serial::stop_bits(serial::stop_bits::one));
    socket().set_option(serial::parity(serial::parity::none));
    socket().set_option(serial::flow_control(serial::flow_control::none));

    // Kick off the session.
    start();
    return true;
  }

  void connect_with_reconnection(bool log_errors = true) {
    if (!attempt_connection(log_errors)) {  
      if (log_errors) {
        ROS_INFO_STREAM("Attempting reconnection every " << interval_.total_milliseconds() << " ms.");
      }
      timer_.expires_from_now(interval_);
      timer_.async_wait(boost::bind(&SerialSession::connect_with_reconnection, this, false));
    } else {
    }
  }

  std::string port_;
  int baud_;
  boost::posix_time::time_duration interval_;
  boost::asio::deadline_timer timer_;
};


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

  new SerialSession(io_service, port, baud);
  io_service.run();

  return 0;
}
