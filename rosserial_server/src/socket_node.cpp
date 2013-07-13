#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

using boost::asio::ip::tcp;

template<typename AsyncReadStream>
class AsyncReadBuffer
{
public:
  AsyncReadBuffer(AsyncReadStream& s, size_t capacity)
       : stream_(s), start_(0), size_(0) {
    mem_.resize(capacity);
  }

  void read(size_t read_count, boost::function<void(char*)> callback) {
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
               boost::function<void(char*)> callback, size_t read_count) {
    if (!error) {
      // TODO: handle error code.

      size_ += bytes_transferred;

      if (callback) {
        size_t read = start_;
        start_ += read_count;
        size_ -= read_count;
        if (size_ == 0) {
          start_ = 0;
        }
        //printf("[%ld,%ld]", start_, size_);
        callback(&mem_[read]);
      } else {
        // TODO: handle bad callback.
      }
    }
  }

  size_t headroom() {
    return mem_.size() - start_ - size_;
  }

  AsyncReadStream& stream_;

  std::vector<char> mem_;

  size_t start_;  // Index of next char to be removed/returned
  size_t size_;  // Number of bytes in buffer at present.
};

class Session
{
public:
  Session(boost::asio::io_service& io_service)
    : socket_(io_service), buffer_(socket_, 1024)
  {
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start()
  {
    std::cout << "Starting\n";
    const char* request_topics = "\xff\xff\x00\x00\x00\x00\xff";
    boost::asio::async_write(socket_, boost::asio::buffer(request_topics, sizeof(request_topics)),
          boost::bind(&Session::handle_write, this, boost::asio::placeholders::error));
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

private:
  void read_sync_first(char* buffer_ptr) {
    printf("x %d \n", buffer_ptr[0]);
    if (buffer_ptr[0] == -1) {
      buffer_.read(1, boost::bind(&Session::read_sync_second, this, _1));
    } else {
      buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
    }
  }
  
  void read_sync_second(char* buffer_ptr) {
    if (buffer_ptr[0] == -1) {
      buffer_.read(4, boost::bind(&Session::read_id_length, this, _1));
    } else {
      buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
    }
  }

  void read_id_length(char* buffer_ptr) {
    uint16_t topic_id = *(uint16_t*)(buffer_ptr + 0);
    uint16_t length = *(uint16_t*)(buffer_ptr + 2);
    printf("rx %d %d\n", topic_id, length);

    buffer_.read(length + 1, boost::bind(&Session::read_message, this,
                                         _1, topic_id, length));
  }

  void read_message(char* buffer_ptr, uint16_t topic_id, uint16_t length) {
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

  tcp::socket socket_;
  AsyncReadBuffer<tcp::socket> buffer_;
  enum { buffer_max = 1023 };
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
