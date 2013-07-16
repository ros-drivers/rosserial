
#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <topic_tools/shape_shifter.h>


class Publisher {
public:
  Publisher(ros::NodeHandle& nh, const rosserial_msgs::TopicInfo& topic_info) {
    message_.morph(topic_info.md5sum, topic_info.message_type, "", "false");
    publisher_ = message_.advertise(nh, topic_info.topic_name, 1);
  }

  void handle(ros::serialization::IStream stream) {
    ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, message_);
    publisher_.publish(message_);
  }

private:
  ros::Publisher publisher_;
  topic_tools::ShapeShifter message_;
};


class Subscriber {
public:
  Subscriber(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info)
      : topic_id_(topic_info.topic_id) {
    ros::SubscribeOptions opts;
    opts.init<topic_tools::ShapeShifter>(
        topic_info.topic_name, 1, boost::bind(&Subscriber::handle, this, _1));
    opts.md5sum = topic_info.md5sum;
    opts.datatype = topic_info.message_type;
    subscriber_ = nh.subscribe(opts);
  }

private:
  void handle(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
    // Called on asynchronous ROS thread.
    // ros::serialization::Serializer<topic_tools::ShapeShifter>::read(stream, message_);
    // publisher_.publish(message_);
  }

  ros::Subscriber subscriber_;
  uint16_t topic_id_;
  //topic_tools::ShapeShifter message_;
};
