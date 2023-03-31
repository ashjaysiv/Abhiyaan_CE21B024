#include "ros/ros.h"
#include "std_msgs/String.h"

std_msgs::String message;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	//message received from the publisher node is saved ("No.1 CFI TEAM")
	message.data  = msg->data.c_str();

}

int main(int argc, char **argv)
{
	//initialize
	ros::init(argc, argv, "subscriber");
	
	//create node variable
	ros::NodeHandle n;
	
	//create subscriber node - /chatter
	ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);
	
	// create a publisher node - /absolute_truth
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("absolute_truth", 1);

	ros::Rate loop_rate(10);	

	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg1;
		std::stringstream ss;
		ss<<message<<"ABHIYAAN";
		msg1.data = ss.str();
		
		//publisher publishes the msg "NO.1 CFI TEAM" + "ABHIYAAN" to the node "absolute_truth"
		ROS_INFO("%s", msg1.data.c_str());

		chatter_pub.publish(msg1);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}


	ros::spin();
	return 0;
}
