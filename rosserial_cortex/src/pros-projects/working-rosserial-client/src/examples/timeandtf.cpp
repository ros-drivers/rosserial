/* 
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

void loop(ros::NodeHandle & nha, geometry_msgs::TransformStamped & ta, tf::TransformBroadcaster & ba)
{  
  char base_link[] = "/base_link";
  char odom[] = "/odom";
  ta.header.frame_id = odom;
  ta.child_frame_id = base_link;
  ta.transform.translation.x = 1.0; 
  ta.transform.rotation.x = 0.0;
  ta.transform.rotation.y = 0.0; 
  ta.transform.rotation.z = 0.0; 
  ta.transform.rotation.w = 1.0;  
  ta.header.stamp = nha.now();
  ba.sendTransform(ta);
  nha.spinOnce();
  delay(10);
}

void setup()
{


  ros::NodeHandle  nh;

  geometry_msgs::TransformStamped t;
  tf::TransformBroadcaster broadcaster;

  nh.initNode();
  broadcaster.init(nh);
  while(1) {
    loop(nh, t, broadcaster);
  }
}
