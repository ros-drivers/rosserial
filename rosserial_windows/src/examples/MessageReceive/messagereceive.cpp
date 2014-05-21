#include "stdafx.h"
#include <string>
#include <stdio.h>
#include "ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <windows.h>

using std::string;

void estimated_pose_callback(const geometry_msgs::PoseWithCovarianceStamped& pose) {
        printf("Received pose %f, %f, %f\n", pose.pose.pose.position.x,
                pose.pose.pose.position.y, pose.pose.pose.position.z);
}

int _tmain(int argc, _TCHAR* argv[])
{
        ros::NodeHandle nh;
        char* ros_master = "1.2.3.4";

        printf("Connecting to server at %s\n", ros_master);
        nh.initNode(ros_master);

        ros::Subscriber <geometry_msgs::PoseWithCovarianceStamped> 
                poseSub("estimated_pose", &estimated_pose_callback);
        nh.subscribe(poseSub);

        printf("Waiting to receive messages\n");
        while (1) {
                nh.spinOnce();
                Sleep(100);
        }

        printf("All done!\n");
        return 0;
}
