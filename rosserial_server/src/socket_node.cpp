#include <cstdlib>
#include <iostream>
#include <tr1/unordered_map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <topic_tools/shape_shifter.h>

using boost::asio::ip::tcp;

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

class Session
{
public:
  Session(boost::asio::io_service& io_service)
    : socket_(io_service), buffer_(socket_, buffer_max)
  {
    //callbacks.insert(std::pair0, boost::bind(&Session::setup_publisher, this)}})
    callbacks[0] = boost::bind(&Session::setup_publisher, this, _1);
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start()
  {
    std::cout << "Starting\n";



    request_topics();
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

private:
  void request_topics() {
    const char* msg = (char*)"\xff\xff\x00\x00\x00\x00\xff";
    boost::asio::async_write(socket_, boost::asio::buffer(msg, sizeof(msg)),
          boost::bind(&Session::write_cb, this, boost::asio::placeholders::error));
  }

  void read_sync_first(ros::serialization::IStream stream) {
    uint8_t sync;
    stream >> sync;
    if (sync == 0xff) {
      buffer_.read(1, boost::bind(&Session::read_sync_second, this, _1));
    } else {
      buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
    }
  }
  
  void read_sync_second(ros::serialization::IStream stream) {
    uint8_t sync;
    stream >> sync;
    if (sync == 0xff) {
      buffer_.read(4, boost::bind(&Session::read_id_length, this, _1));
    } else {
      buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
    }
  }

  void read_id_length(ros::serialization::IStream stream) {
    uint16_t topic_id, length;
    stream >> topic_id >> length;
    // printf("rx %d %d\n", topic_id, length);
    // TODO: Check length against expected for fixed-length messages.

    buffer_.read(length + 1, boost::bind(&Session::read_message, this,
                                         _1, topic_id));
  }

  void read_message(ros::serialization::IStream stream, uint16_t topic_id) {
    // TODO: Check checksum here.
    
    if (callbacks.count(topic_id) == 1) {
      callbacks[topic_id](stream);
    } else {
      // TODO: Handle unrecognized topic_id.
    }

    // Kickoff next message read.
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

  void write_cb(const boost::system::error_code& error) { 
    if (error) {
      delete this;
    }
  }

  void setup_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo msg;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, msg);
    std::cout << msg;
    //publishers[] =
    //callback[] = 
  }

  tcp::socket socket_;
  AsyncReadBuffer<tcp::socket> buffer_;
  enum { buffer_max = 1023 };

  std::tr1::unordered_map< uint16_t, boost::function<void(ros::serialization::IStream)> > callbacks;
};

class Server
{
public:
  Server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    Session* new_session = new Session(io_service_);
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&Server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }

  void handle_accept(Session* new_session,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      new_session->start();
    }
    else
    {
      delete new_session;
    }

    start_accept();
  }

  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
};

int main(int argc, char* argv[])
{
  try
  {
    int port = 11411;
    boost::asio::io_service io_service;
    Server s(io_service, port);

    std::cout << "Listening on port " << port << "\n";
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
