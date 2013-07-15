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


