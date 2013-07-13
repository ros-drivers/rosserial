
#include <ros/ros.h>
#include <rosserial_msgs/TopicInfo.h>
#include <topic_tools/shape_shifter.h>


class Publisher {
public:
  Publisher(ros::NodeHandle& nh, rosserial_msgs::TopicInfo& topic_info) {
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
