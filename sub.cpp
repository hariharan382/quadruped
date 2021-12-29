#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <cstring>
#include <string>

void call_back(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("I heard: [%i]", msg->data);
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"subscriber_node");
	ros::NodeHandle nh;
	ros::Subscriber sub_obj=nh.subscribe("angle_topic",1000,call_back);
	ros::spin();
	
	return 0;
	
}

