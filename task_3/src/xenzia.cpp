#include <ros/ros.h> 
#include <geometry_msgs/Twist.h> //msg of topic cmd_vel
#include <geometry_msgs/Pose.h> //msg of topic pose
#include <turtlesim/Pose.h> //topic cmd_vel
#include <turtlesim/Spawn.h> // sevice spawn
#include <turtlesim/Kill.h> //service kill
#include <math.h> //atan2

using namespace std;

//GLOBAL VARIABLES

turtlesim::Pose turtle_pose; //position

ros::Publisher cmd_vel_pub; //publish velocity
ros::Subscriber pose_sub; //subscribe position

ros::ServiceClient spawn_client; //spawn
ros::ServiceClient kill_client; //kill

//req and resp for spawn
turtlesim::Spawn::Request spawn_req;
turtlesim::Spawn::Response spawn_resp;

//re and resp for spwan
turtlesim::Kill::Request kill_req;
turtlesim::Kill::Request kill_resp;

//FUNCTION PROTOTYPES
void poseCallBack(const turtlesim::Pose::ConstPtr &pose_msg);
double linear_vel(turtlesim::Pose final_pose, turtlesim::Pose current_pose);
double angular_vel(turtlesim::Pose final_pose, turtlesim::Pose current_pose);
void moveTo(turtlesim::Pose final_pose, double tolerance);
void spawnTurtle(turtlesim::Pose leader_pose);
void killTurtle(void);

//MAIN

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xenzia");
	ros::NodeHandle n;
	
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallBack);
	spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");
	kill_client = n.serviceClient<turtlesim::Kill>("/kill");
	
	turtlesim::Pose goal_pose; //goal position
	
	while(true)
	{
		goal_pose.x = random() % 5;
		goal_pose.y = random() % 5;
		
		spawnTurtle(goal_pose);
		moveTo(goal_pose, 0.01);
		killTurtle();
	}
	
	return 0;
}
		
//SUPPORT FUNCTIONS

//get position of newly spawned turtle
void poseCallBack(const turtlesim::Pose::ConstPtr &pose_msg)
{
	turtle_pose = *pose_msg;
}

//calculate linear velocity
double linear_vel(turtlesim::Pose final_pose, turtlesim::Pose current_pose)
{
	double euclidean_distance;
	//distance between current state and final state
	euclidean_distance = sqrt(pow((final_pose.x - current_pose.x), 2) + pow((final_pose.y - current_pose.y), 2));
	
	//proportional controller
	return 1.5 * euclidean_distance; 
}

//calculate angular velocity
double angular_vel(turtlesim::Pose final_pose, turtlesim::Pose current_pose)
{
	double steering_angle;
	//slope of line joining the hunter and the prey
	steering_angle = atan2((final_pose.y - current_pose.y) ,(final_pose.x - current_pose.y));
	
	//proportional controller
	return 3 * (steering_angle - current_pose.theta); 
}

//move hunter to prey
void moveTo(turtlesim::Pose final_pose, double tolerance)
{
	geometry_msgs::Twist cmd_vel_msg;
	ros::Rate loop_rate(10);
	
	double distance;
	
	do {
		//x-direction linear velocity
		cmd_vel_msg.linear.x = linear_vel(final_pose, turtle_pose);
		cmd_vel_msg.angular.z = angular_vel(final_pose, turtle_pose);
		cmd_vel_pub.publish(cmd_vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
		
		
	} while (distance >= tolerance);
	
	//stop
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.angular.z = 0;
	cmd_vel_pub.publish(cmd_vel_msg);
}

void spawnTurtle(turtlesim::Pose leader_pose)
{
	spawn_req.x = leader_pose.x;
	spawn_req.y = leader_pose.y;
	spawn_req.theta = leader_pose.theta;
	spawn_req.name = "Leader";
	ros::service::waitForService("/spawn", ros::Duration(5));
	bool success = spawn_client.call(spawn_req, spawn_resp);
	
	if(success){
		ROS_INFO_STREAM("Reborn like fenix turtle named"<< spawn_resp.name);
	}
	
	else{
		ROS_ERROR_STREAM("Failed to spawn turtle named" << spawn_resp.name);
	}
}

void killTurtle(void)
{
	kill_req.name = spawn_resp.name;
	ros::service::waitForService("/kill", ros::Duration(5));
	bool success = kill_client.call(kill_req, kill_resp);
	if(success){
		ROS_INFO_STREAM("Killed beloved turtle named"<< spawn_resp.name);
	}
	
	else{
		ROS_ERROR_STREAM("Failed to kill turtle named" << spawn_resp.name);
	}
}
	
	
	
