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
       : stream_(s), start_(0), size_(0), error_callback_(error_callback) {
    mem_.resize(capacity);
  }

  void read(size_t read_count, boost::function<void(ros::serialization::IStream&)> callback) {
    // Objective is to ensure that the buffer contains number of bytes,
    // contiguously, that the caller wishes.
    // If it does already, then call the callback immediately. If not, then
    // command an asynchronous read on the socket, which will call
    // the callback.
    ROS_DEBUG("Requested %ld bytes, %ld bytes available in buffer.", read_count, size_);
    if (read_count <= size_) {
      read_cb(boost::system::errc::make_error_code(boost::system::errc::success), 
              0, callback, read_count);
      return;
    }

    /*if (size_ == 0) {
      // Buffer is presently empty. Take advantage of the opportunity to reset
      // to the start of the buffer while the copy cost is zero.
      start_ = 0;
    }*/

    if (read_count > mem_.size()) {
      // Insufficient room in the buffer for the requested bytes,
      ROS_ERROR("Requested to read %ld bytes, but buffer capacity is only %ld.", read_count, mem_.size());
      error_callback_(boost::system::errc::make_error_code(boost::system::errc::no_buffer_space));
    }

    if (read_count > size_ + headroom()) {
      // Shift buffer contents back to beginning point so that we have room for requested read.
      ROS_DEBUG("Buffer contains %ld bytes with %ld headroom, caller has requested %ld bytes.", size_, headroom(), read_count);
      memmove(&mem_[0], &mem_[start_], size_);
      start_ = 0;
      memset(&mem_[size_], 3, headroom());
      ROS_DEBUG("Buffer shifted, contains %ld bytes now with %ld headroom.", size_, headroom());
    }

    ROS_DEBUG("Requesting transfer of %ld bytes, to fill user request of %ld bytes.", read_count - size_, read_count);
    boost::asio::async_read(stream_,
        boost::asio::buffer(&mem_[start_ + size_], headroom()),
        boost::asio::transfer_at_least(read_count - size_),
        boost::bind(&AsyncReadBuffer::read_cb, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred,
                    callback, read_count));
  }

private:
  void read_cb(const boost::system::error_code& error, size_t bytes_transferred,
               boost::function<void(ros::serialization::IStream&)> callback, size_t read_count) {
    if (error) {
      if (error_callback_) {
        error_callback_(error);
      }
    } else {
      size_ += bytes_transferred;
      ROS_DEBUG_COND(bytes_transferred > 0, "Transferred %ld bytes, buffer size now %ld.", bytes_transferred, size_);

      if (callback) {
        ROS_DEBUG("Preparing input stream, starts at index %ld, size %ld.", start_, read_count);
        ros::serialization::IStream stream(&mem_[start_], read_count);
        start_ += read_count;
        size_ -= read_count;
        callback(stream);
      } else {
        // TODO: handle bad callback.
      }
    }
  }

  size_t headroom() {
    return mem_.size() - start_ - size_ -1;
  }

  AsyncReadStream& stream_;

  std::vector<uint8_t> mem_;

  size_t start_;  // Index of next char to be removed/returned
  size_t size_;  // Number of bytes in buffer at present.
                  
  boost::function<void(const boost::system::error_code&)> error_callback_;
};


