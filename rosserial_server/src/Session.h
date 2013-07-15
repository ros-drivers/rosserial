#include <tr1/unordered_map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Time.h>

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
    callbacks_[rosserial_msgs::TopicInfo::ID_PUBLISHER]
        = boost::bind(&Session::setup_publisher, this, _1);
    //callbacks_[rosserial_msgs::TopicInfo::ID_SUBSCRIBER] = boost::bind(&Session::setup_subscriber, this, _1);
    callbacks_[rosserial_msgs::TopicInfo::ID_TIME]
        = boost::bind(&Session::handle_time, this, _1);

  }

  ~Session()
  {
    ROS_INFO("Ending session.");  
  }

  Socket& socket()
  {
    return socket_;
  }

  void start()
  {
    ROS_INFO("Starting session.");

    request_topics();
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

private:
  void request_topics() {
    std::vector<uint8_t> message(0);
    write_message(message, rosserial_msgs::TopicInfo::ID_PUBLISHER);
  }

  void read_failed(const boost::system::error_code& error) {
    if (error) {
      // When a read error has occurred, delete the whole session, which destroys
      // all publishers and subscribers.
      delete this;
    }
  }

  void read_sync_first(ros::serialization::IStream& stream) {
    uint8_t sync;
    stream >> sync;
    if (sync == 0xff) {
      buffer_.read(1, boost::bind(&Session::read_sync_second, this, _1));
    } else {
      buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
    }
  }
  
  void read_sync_second(ros::serialization::IStream& stream) {
    uint8_t sync;
    stream >> sync;
    if (sync == 0xff) {
      buffer_.read(4, boost::bind(&Session::read_id_length, this, _1));
    } else {
      buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
    }
  }

  void read_id_length(ros::serialization::IStream& stream) {
    uint16_t topic_id, length;
    stream >> topic_id >> length;
    // printf("rx %d %d\n", topic_id, length);
    // TODO: Check length against expected for known fixed-length messages.

    buffer_.read(length + 1, boost::bind(&Session::read_body, this,
                                         _1, topic_id));
  }


  void read_body(ros::serialization::IStream& stream, uint16_t topic_id) {
    ROS_DEBUG("Received body of length %d for message on topic %d.", stream.getLength(), topic_id);
    
    ros::serialization::IStream checksum_stream(stream.getData(), stream.getLength());
    if (message_checksum(checksum_stream, topic_id) != 0xff) {
      ROS_WARN("Received message on topicId=%d, length=%d with bad checksum.", topic_id, stream.getLength());
    } else {
      if (callbacks_.count(topic_id) == 1) {
        callbacks_[topic_id](stream);
      } else {
        ROS_WARN("Received message with unrecognized topicId (%d).", topic_id);
        // TODO: Resynchronize?
      }
    }

    // Kickoff next message read.
    buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

  void write_message(std::vector<uint8_t>& message, const uint16_t topic_id) {
    uint16_t length = 7 + message.size();
    boost::shared_ptr< std::vector<uint8_t> > buffer_ptr(new std::vector<uint8_t>(length));

    //const uint8_t* message_data_ptr = 0;
    ros::serialization::IStream checksum_stream(
        message.size() > 0 ? &message[0] : NULL, message.size());
    uint8_t checksum = message_checksum(checksum_stream, topic_id);

    ros::serialization::OStream stream(&buffer_ptr->at(0), buffer_ptr->size()); 
    stream << (uint16_t)0xffff << topic_id << (uint16_t)message.size();
    memcpy(stream.advance(message.size()), &message[0], message.size());
    stream << checksum;

    boost::asio::async_write(socket_, boost::asio::buffer(*buffer_ptr),
          boost::bind(&Session::write_cb, this, boost::asio::placeholders::error, buffer_ptr));
  }

  void write_cb(const boost::system::error_code& error,
                boost::shared_ptr< std::vector<uint8_t> > buffer_ptr) { 
    if (error) {
      delete this;
    }
    // Buffer is destructed when this function exits.
  }

  //// HELPERS ////

  static uint8_t message_checksum(ros::serialization::IStream& stream, const uint16_t topic_id) {
    uint8_t sum = (topic_id >> 8) + topic_id + (stream.getLength() >> 8) + stream.getLength();
    for (uint16_t i = 0; i < stream.getLength(); ++i) {
      sum += stream.getData()[i];
    }
    return 255 - sum;
  }

  //// HANDLERS ////

  void setup_publisher(ros::serialization::IStream& stream) {
    rosserial_msgs::TopicInfo topic_info;
    ros::serialization::Serializer<rosserial_msgs::TopicInfo>::read(stream, topic_info);
    //std::cout << topic_info;

    uint16_t t_id = topic_info.topic_id;
    boost::shared_ptr<Publisher> pub(new Publisher(nh_, topic_info));
    callbacks_[t_id] = boost::bind(&Publisher::handle, pub, _1);
    publishers_[t_id] = pub;
  }

  void handle_time(ros::serialization::IStream& stream) {
    std_msgs::Time time;
    time.data = ros::Time::now();

    size_t length = ros::serialization::serializationLength(time);
    std::vector<uint8_t> message(length);

    ros::serialization::OStream ostream(&message[0], length);
    ros::serialization::Serializer<std_msgs::Time>::write(ostream, time); 
 
    write_message(message, rosserial_msgs::TopicInfo::ID_TIME);
  }

  Socket socket_;
  AsyncReadBuffer<Socket> buffer_;
  enum { buffer_max = 1023 };
  ros::NodeHandle nh_;

  std::map< uint16_t, boost::function<void(ros::serialization::IStream)> > callbacks_;
  std::map< uint16_t, boost::shared_ptr<Publisher> > publishers_;
};


