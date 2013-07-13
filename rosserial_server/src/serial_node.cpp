#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class session
{
public:
  session(boost::asio::io_service& io_service)
    : socket_(io_service)
  {
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start()
  {
    std::cout << "Starting\n";
    const char* sync_msg = "\xff\xff\x00\x00\x00\x00\xff";
    boost::asio::async_write(socket_, boost::asio::buffer(sync_msg, sizeof(sync_msg)),
          boost::bind(&session::handle_write, this, boost::asio::placeholders::error));
    /*socket_.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&session::handle_read, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));*/
  }

private:
  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    std::cout << "RX " << bytes_transferred << "  " << error << "\n";
    if (!error)
    {
      boost::asio::async_read(socket_, boost::asio::buffer(data_, max_length),
          boost::asio::transfer_at_least(21),
          boost::bind(&session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
      /*boost::asio::async_write(socket_,
          boost::asio::buffer(data_, bytes_transferred),
          boost::bind(&session::handle_write, this,
            boost::asio::placeholders::error));*/
    }
    else
    {
      delete this;
    }
  }

  void handle_write(const boost::system::error_code& error)
  {
    std::cout << "Written "<< error << "\n";
    if (!error)
    {
      boost::asio::async_read(socket_, boost::asio::buffer(data_, max_length),
          boost::asio::transfer_at_least(16),
          boost::bind(&session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      delete this;
    }
  }

  tcp::socket socket_;
  enum { max_length = 2012 };
  char data_[max_length];
};

class server
{
public:
  server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    session* new_session = new session(io_service_);
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }

  void handle_accept(session* new_session,
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
    server s(io_service, port);

    std::cout << "Listening on port " << port << "\n";
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
