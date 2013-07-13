#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "Session.h"


using boost::asio::ip::tcp;
typedef Session<tcp::socket> SocketSession;

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
    SocketSession* new_session = new SocketSession(io_service_);
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&Server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }

  void handle_accept(SocketSession* new_session,
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
