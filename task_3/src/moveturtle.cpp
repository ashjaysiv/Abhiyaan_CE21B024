#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <math.h>

using namespace std;

turtlesim::Pose turtle_pose;
ros::Publisher cmd_vel_pub;
ros::Subscriber pose_sub;

ros::ServiceClient spawn_client;
ros::ServiceClient kill_client;

turtlesim::Spawn::Request spawn_req;
turtlesim::Spawn::Response spawn_resp;

turtlesim::Kill::Request kill_req;
turtlesim::Kill::Response kill_resp;

void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg);
void moveTo(turtlesim::Pose goal_pose, double tolerance);
double PID_distance(std::vector<double> ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);
double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose,turtlesim::Pose turtle_pose, double dt);
void spawnTurtle(turtlesim::Pose leader_pose);
void killTurtle(void);
bool nearwall(turtlesim::Pose setpoint_pose);

int main(int argc, char **argv){

	ros::init(argc, argv, "moveTurtle");
	ros::NodeHandle n;
	
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);
	spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");
	kill_client = n.serviceClient<turtlesim::Kill>("/kill");
	
	turtlesim::Pose goal_pose;
	while(true){
	goal_pose.x = 1.01 + random() % 8;
	goal_pose.y = 1.01 + random() % 8;
	spawnTurtle(goal_pose);
	moveTo(goal_pose, 0.1);
	killTurtle();
	}
	return 0;
	}
	
bool nearwall(turtlesim::Pose setpoint_pose){
	if(setpoint_pose.x < 1 || setpoint_pose.x> 9.8 || setpoint_pose.y < 1 || setpoint_pose.y > 9.8)
		{
			return true;
		}
		
	else{
		return false;
		}
}	

bool hitwall(turtlesim::Pose setpoint_pose){
	if(floor(setpoint_pose.x) == 11 || floor(setpoint_pose.x) == 0 || floor(setpoint_pose.y) == 0 || floor(setpoint_pose.y) == 11){
		return true;
	}  
	return false;
}
void moveTo(turtlesim::Pose finish_pose, double tolerance){
	geometry_msgs::Twist cmd_vel_msg;
	ros::Rate loop_rate(10);
	vector<double> KpKiKd_distance{1.5, 0, 0};
	vector<double> KpKiKd_angle{3,0,0};
	double d;
	double dt;
	ros::Time start = ros::Time::now();
	ros::Time end = ros::Time::now() + ros::Duration(0.1);
	do{
		if (nearwall(turtle_pose))
		{
			cmd_vel_msg.linear.x = -cmd_vel_msg.linear.x;
			cmd_vel_msg.angular.x = 3.14 - cmd_vel_msg.angular.x;
			cmd_vel_pub.publish(cmd_vel_msg);
			}
			
		//if (nearwall(turtle_pose))
		//{
//			kill_req.name = "turtle1";
//			ros::service::waitForService("/kill", ///ros::Duration(5));
//			bool success = kill_client.call(kill_req, kill_resp);
//	if(success){
//		ROS_INFO_STREAM("Killed beloved turtle named"<< kill_req.name);
//	}
	
//	else{
//		ROS_ERROR_STREAM("Failed to kill turtle named" << kill_req.name);
//		}
//	}
		end = ros::Time::now();
		dt = end.toSec() - start.toSec();
		dt = dt < (1.0/10)*(0.05)?0.1:dt;
		d = PID_distance(KpKiKd_distance, finish_pose, turtle_pose, 1.0/10);
		start = ros::Time::now();
		cmd_vel_msg.linear.x = d;
		cmd_vel_msg.angular.z = PID_angle(KpKiKd_angle, finish_pose, turtle_pose, 1.0/10);
		
		
		cmd_vel_pub.publish(cmd_vel_msg);
		
		ros::spinOnce();
		}while (d>tolerance);
		
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.angular.z = 0;
	cmd_vel_pub.publish(cmd_vel_msg);
	}
	
void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg){
	turtle_pose = *pose_msg;
	}

double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt){
	turtlesim::Pose e;
	static double derror;
	static double prev_error;
	
	double Kd = Ks.back(); Ks.pop_back();
	double Ki = Ks.back(); Ks.pop_back();
	double Kp = Ks.back(); Ks.pop_back();
	
	e.x = setpoint_pose.x - turtle_pose.x;
	e.y = setpoint_pose.y - turtle_pose.y;
	double error = sqrt(pow(e.x,2)  + pow(e.y, 2));
	derror = error - prev_error;
	double u = Kp*error + Ki*error*dt + Kd*derror/dt;
	prev_error = error;
	return u;
}

double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt){
	turtlesim::Pose e;
	static double derror;
	static double prev_error;
	
	double Kd = Ks.back(); Ks.pop_back();
	double Ki = Ks.back(); Ks.pop_back();
	double Kp = Ks.back(); Ks.pop_back();
	
	double error = atan2(setpoint_pose.y - turtle_pose.y, setpoint_pose.x - turtle_pose	.x) - turtle_pose.theta;
	derror = error - prev_error;
	double u = Kp*error + Ki*error*dt + Kd*derror/dt;
	
	prev_error = error;
	
	u = u > 4*M_PI? 4*M_PI:u;
	u = u < -4*M_PI? -4*M_PI:u;
	
	return u;
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

	
	
		
