#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "Session.h"
#include "AsyncOkPoll.h"


using boost::asio::ip::tcp;
//typedef Session<tcp::socket> SocketSession;


template<typename Session>
class TcpServer
{
public:
  TcpServer(boost::asio::io_service& io_service, short port)
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
        boost::bind(&TcpServer::handle_accept, this, new_session,
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
  boost::asio::io_service io_service;

  // Initialize ROS.
  ros::init(argc, argv, "rosserial_server_socket_node");
  ros::AsyncSpinner ros_spinner(1);
  ros_spinner.start();

  // Monitor ROS for shutdown, and stop the io_service accordingly.
  AsyncOkPoll ok_poll(io_service, boost::posix_time::milliseconds(500), ros::ok);

  // Start listening for rosserial TCP connections.
  int port = 11411;
  TcpServer< Session<tcp::socket> > s(io_service, port);
  std::cout << "Listening on port " << port << "\n";
  io_service.run();

  return 0;
}
