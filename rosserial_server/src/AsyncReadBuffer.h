#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>


template<typename AsyncReadStream>
class AsyncReadBuffer
{
public:
  AsyncReadBuffer(AsyncReadStream& s, size_t capacity)
       : stream_(s), start_(0), size_(0) {
    mem_.resize(capacity);
  }

  void read(size_t read_count, boost::function<void(ros::serialization::IStream)> callback) {
    // Objective is to ensure that the buffer contains number of bytes,
    // contiguously, that the caller wishes.
    // If it does already, then call the callback immediately. If not, then
    // command an asynchronous read on the socket, which will call
    // the callback.
    if (read_count <= size_) {
      read_cb(boost::system::errc::make_error_code(boost::system::errc::success), 
              0, callback, read_count);
      return;
    }

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
               boost::function<void(ros::serialization::IStream)> callback, size_t read_count) {
    if (error) {
      // TODO: handle error code.
    } else {
      size_ += bytes_transferred;

      if (callback) {
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
    return mem_.size() - start_ - size_;
  }

  AsyncReadStream& stream_;

  std::vector<uint8_t> mem_;

  size_t start_;  // Index of next char to be removed/returned
  size_t size_;  // Number of bytes in buffer at present.
};


