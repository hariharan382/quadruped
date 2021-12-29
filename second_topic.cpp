#include <ros/ros.h>
#include <std_msgs/Int32.h>


int main(int argc, char **argv)
{
	ros::init(argc,argv,"angle_publishing_node_2");
	ros::NodeHandle nh;
	ros::Publisher pub_1 = nh.advertise<std_msgs::Int32>("angle_topic2",10);	
		while(ros::ok())
	{
		ros::spinOnce();
	}
	
	return 0;
	
}
