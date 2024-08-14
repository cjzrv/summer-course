#include "ros/ros.h"
#include "std_msgs/Int32.h"

ros::Publisher pub;

void callback(const std_msgs::Int32::ConstPtr& msg)
{
    std_msgs::Int32 temp;
    temp.data = msg->data + 1;
    ROS_INFO("c = b+1 = %d+1 = %d", msg->data, temp.data);
    ros::Duration(0.5).sleep();
    pub.publish(temp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::Int32>("topic_c", 1000);
    ros::Subscriber sub = nh.subscribe("topic_b", 1000, callback);

    ros::spin();
    return 0;
}
