/**
 *
 *  \file
 *  \brief      Helper object for successive reads from a ReadStream.
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

#ifndef ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H
#define ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>

namespace rosserial_server
{

template<typename AsyncReadStream>
class AsyncReadBuffer
{
public:
  AsyncReadBuffer(AsyncReadStream& s, size_t capacity,
                  boost::function<void(const boost::system::error_code&)> error_callback)
       : stream_(s), error_callback_(error_callback) {
    mem_.resize(capacity);
    ROS_ASSERT_MSG(error_callback_, "Bad error callback passed to read buffer.");
  }

  void read(size_t read_count, boost::function<void(ros::serialization::IStream&)> callback) {
     if (read_count > mem_.size()) {
      // Insufficient room in the buffer for the requested bytes,
      ROS_ERROR_STREAM_NAMED("async_read", "Requested to read " << read_count << " bytes, but buffer capacity is only " << mem_.size() << ".");
      error_callback_(boost::system::errc::make_error_code(boost::system::errc::no_buffer_space));
      return;
    }

    boost::asio::async_read(stream_,
        boost::asio::buffer(&mem_[0], read_count),
        boost::asio::transfer_at_least(read_count),
        boost::bind(&AsyncReadBuffer::read_cb, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred,
                    callback));
  }

private:
  void read_cb(const boost::system::error_code& error, size_t bytes_transferred,
               boost::function<void(ros::serialization::IStream&)> callback) {
    if (error)
    {
      if (error == boost::asio::error::operation_aborted)
      {
        // Special case for operation_aborted. The abort callback comes when the owning Session
        // is in the middle of teardown, which means the callback is no longer valid and calling
        // it would be a segfault.
      }
      else
      {
        error_callback_(error);
      }
    } else {
      ROS_DEBUG_STREAM_NAMED("async_read", "Transferred " << bytes_transferred << " byte(s).");

      ros::serialization::IStream stream(&mem_[0], bytes_transferred);
      ROS_ASSERT_MSG(callback, "Bad read callback function.");
      callback(stream);
    }
  }

  AsyncReadStream& stream_;
  std::vector<uint8_t> mem_;
  boost::function<void(const boost::system::error_code&)> error_callback_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H
