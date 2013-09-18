/**
 *
 *  \file
 *  \brief      Periodic asio callback to check for an exit condition.
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

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>


class AsyncOkPoll {
public:
  AsyncOkPoll(boost::asio::io_service& io_service,
                    boost::posix_time::time_duration interval,
                    boost::function<bool()> ok_fn)
      : io_service_(io_service), interval_(interval), fn_(ok_fn),
        timer_(io_service) {
    if (!fn_) {
      return;
    }
    set();
  }

private:
  void set() {
    timer_.expires_from_now(interval_);
    timer_.async_wait(boost::bind(&AsyncOkPoll::handler, this,
          boost::asio::placeholders::error));
  }

  void handler(const boost::system::error_code& error) {
    if (!error) {
      if (!fn_()) {
        io_service_.stop();
      } else {
        set();
      }
    }
  }

  boost::posix_time::time_duration interval_;
  boost::asio::deadline_timer timer_;
  boost::asio::io_service& io_service_;
  boost::function<bool()> fn_;
};


