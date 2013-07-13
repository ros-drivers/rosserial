#include <tr1/unordered_map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <topic_tools/shape_shifter.h>

#include "AsyncReadBuffer.h"
#include "topic_handlers.h"


template<typename Socket>
class Session
{
public:
  Session(boost::asio::io_service& io_service)
    : socket_(io_service), 
      buffer_(socket_, buffer_max, 
              boost::bind(&Session::read_failed, this,
                          boost::asio::placeholders::error))
  {
    callbacks_[0] = boost::bind(&Session::setup_publisher, this, _1);
  }

  Socket& socket()
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

  void read_failed(const boost::system::error_code& error) {
    if (error) {
      // When a read error has occurred, delete the whole session, which destroys
      // all publishers and subscribers.
      delete this;
    }
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
    
    if (callbacks_.count(topic_id) == 1) {
      callbacks_[topic_id](stream);
    } else {
      // TODO: Handle unrecognized topic_id.
    }

    // Kickoff next message read.
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

  void write_cb(const boost::system::error_code& error) { 
    boost::asio::io_service io_service;
    if (error) {
      delete this;
    }
  }

  void setup_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);
    std::cout << topic_info;

    uint16_t t_id = topic_info.topic_id;
    boost::shared_ptr<Publisher> pub(new Publisher(nh_, topic_info));
    callbacks_[t_id] = boost::bind(&Publisher::handle, pub, _1);
    publishers_[t_id] = pub;
  }

  Socket socket_;
  AsyncReadBuffer<Socket> buffer_;
  enum { buffer_max = 1023 };
  ros::NodeHandle nh_;

  std::tr1::unordered_map< uint16_t, boost::function<void(ros::serialization::IStream)> > callbacks_;
  std::tr1::unordered_map< uint16_t, boost::shared_ptr<Publisher> > publishers_;
};


