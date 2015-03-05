/**
 *
 *  \file
 *  \brief      Single, reconnecting class for a serial rosserial session.
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

#ifndef ROSSERIAL_SERVER_SERIAL_SESSION_H
#define ROSSERIAL_SERVER_SERIAL_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "rosserial_server/session.h"

namespace rosserial_server
{

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
  ~SerialSession()
  {
    ROS_WARN("Serial session shutting down. Waiting 1 second for system state to settle.");

    boost::shared_ptr<boost::asio::deadline_timer> timer
          (new boost::asio::deadline_timer(
              socket().get_io_service(),
              boost::posix_time::seconds(1)));

    // The timer instance is only passed to the callback in order to keep it alive for the
    // required lifetime. When the callback completes, it goes out of scope and is destructed.
    timer->async_wait(
       boost::bind(&SerialSession::restart_session,
                   boost::ref(socket().get_io_service()), port_, baud_, timer));
  }

  static void restart_session(boost::asio::io_service& io_service, std::string port, int baud,
                              boost::shared_ptr<boost::asio::deadline_timer>& timer)
  {
    if (ros::ok()) {
      ROS_INFO("Recreating serial session.");
      new SerialSession(io_service, port, baud);
    } else {
      ROS_INFO("In shutdown, avoiding recreating serial session.");
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

}  // namespace

#endif  // ROSSERIAL_SERVER_SERIAL_SESSION_H
