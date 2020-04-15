/**
 *
 *  \file
 *  \brief      Classes which manage the Publish and Subscribe relationships
 *              that a Session has with its client.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_TOPIC_HANDLERS_H
#define ROSSERIAL_SERVER_TOPIC_HANDLERS_H

#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <topic_tools/shape_shifter.h>
#include <rosserial_server/msg_lookup.h>

namespace rosserial_server
{

class Publisher {
public:
  Publisher(ros::NodeHandle& nh, const rosserial_msgs::TopicInfo& topic_info) {
    rosserial_server::MsgInfo msginfo;
    try
    {
      msginfo = lookupMessage(topic_info.message_type);
    }
    catch (const std::exception& e)
    {
      ROS_WARN_STREAM("Unable to look up message: " << e.what());
    }

    if (!msginfo.md5sum.empty() && msginfo.md5sum != topic_info.md5sum)
    {
      ROS_WARN_STREAM("Message" << topic_info.message_type  << "MD5 sum from client does not "
                      "match that in system. Will avoid using system's message definition.");
      msginfo.full_text = "";
    }
    message_.morph(topic_info.md5sum, topic_info.message_type, msginfo.full_text, "false");
    publisher_ = message_.advertise(nh, topic_info.topic_name, 1);
  }

  void handle(ros::serialization::IStream stream) {
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, message_);
    publisher_.publish(message_);
  }

  std::string get_topic() {
    return publisher_.getTopic();
  }

private:
  ros::Publisher publisher_;
  topic_tools::ShapeShifter message_;
};

typedef boost::shared_ptr<Publisher> PublisherPtr;


class Subscriber {
public:
  Subscriber(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t>& buffer)> write_fn)
    : write_fn_(write_fn) {
    ros::SubscribeOptions opts;
    opts.init<topic_tools::ShapeShifter>(
        topic_info.topic_name, 1, boost::bind(&Subscriber::handle, this, _1));
    opts.md5sum = topic_info.md5sum;
    opts.datatype = topic_info.message_type;
    subscriber_ = nh.subscribe(opts);
  }

  std::string get_topic() {
    return subscriber_.getTopic();
  }

private:
  void handle(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
    size_t length = ros::serialization::serializationLength(*msg);
    std::vector<uint8_t> buffer(length);

    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

    write_fn_(buffer);
  }

  ros::Subscriber subscriber_;
  boost::function<void(std::vector<uint8_t>& buffer)> write_fn_;
};

typedef boost::shared_ptr<Subscriber> SubscriberPtr;

class ServiceClient {
public:
  ServiceClient(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info,
      boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn)
    : write_fn_(write_fn) {
    topic_id_ = -1;

    rosserial_server::MsgInfo srvinfo;
    rosserial_server::MsgInfo reqinfo;
    rosserial_server::MsgInfo respinfo;
    try
    {
      srvinfo = lookupMessage(topic_info.message_type, "srv");
      reqinfo = lookupMessage(topic_info.message_type + "Request", "srv");
      respinfo = lookupMessage(topic_info.message_type + "Response", "srv");
    }
    catch (const std::exception& e)
    {
      ROS_WARN_STREAM("Unable to look up service definition: " << e.what());
    }
    service_md5_ = srvinfo.md5sum;
    request_message_md5_ = reqinfo.md5sum;
    response_message_md5_ = respinfo.md5sum;

    ros::ServiceClientOptions opts;
    opts.service = topic_info.topic_name;
    opts.md5sum = srvinfo.md5sum;
    opts.persistent = false; // always false for now
    service_client_ = nh.serviceClient(opts);
  }
  void setTopicId(uint16_t topic_id) {
    topic_id_ = topic_id;
  }
  std::string getServiceMD5() {
    return service_md5_;
  }
  std::string getRequestMessageMD5() {
    return request_message_md5_;
  }
  std::string getResponseMessageMD5() {
    return response_message_md5_;
  }

  void handle(ros::serialization::IStream stream) {
    // deserialize request message
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, request_message_);

    // perform service call
    // note that at present, at least for rosserial-windows a service call returns nothing,
    // so we discard the return value of this call() invocation.
    service_client_.call(request_message_, response_message_, service_md5_);

    // write service response over the wire
    size_t length = ros::serialization::serializationLength(response_message_);
    std::vector<uint8_t> buffer(length);
    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, response_message_);
    write_fn_(buffer,topic_id_);
  }

private:
  topic_tools::ShapeShifter request_message_;
  topic_tools::ShapeShifter response_message_;
  ros::ServiceClient service_client_;
  boost::function<void(std::vector<uint8_t>& buffer, const uint16_t topic_id)> write_fn_;
  std::string service_md5_;
  std::string request_message_md5_;
  std::string response_message_md5_;
  uint16_t topic_id_;
};

typedef boost::shared_ptr<ServiceClient> ServiceClientPtr;

}  // namespace

#endif  // ROSSERIAL_SERVER_TOPIC_HANDLERS_H
