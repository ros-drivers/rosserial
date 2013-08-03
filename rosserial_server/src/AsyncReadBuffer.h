#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>


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
      ROS_ERROR_NAMED("async_read", "Requested to read %ld bytes, but buffer capacity is only %ld.", read_count, mem_.size());
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
    if (error) {
      error_callback_(error);
    } else {
      ROS_DEBUG_NAMED("async_read", "Transferred %ld byte(s).", bytes_transferred);

      ros::serialization::IStream stream(&mem_[0], bytes_transferred);
      ROS_ASSERT_MSG(callback, "Bad read callback function.");
      callback(stream);
    }
  }

  AsyncReadStream& stream_;
  std::vector<uint8_t> mem_;
  boost::function<void(const boost::system::error_code&)> error_callback_;
};


