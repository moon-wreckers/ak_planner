#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "rover_state_publisher");
	ros::NodeHandle n;

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1);
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(30);


	//------------------------------------TESTING---------------------------------------

	

	//----------------------------------------------------------------------------------

	const double degree = M_PI/180;

	double angle = 0;

	geometry_msgs::TransformStamped odom_trans_base;
	odom_trans_base.header.frame_id = "odom";
	odom_trans_base.child_frame_id = "base_link";

	geometry_msgs::TransformStamped base_trans_front_right_wheel;
	base_trans_front_right_wheel.header.frame_id = "base_link";
	base_trans_front_right_wheel.child_frame_id = "front_right_wheel";
	base_trans_front_right_wheel.transform.translation.x = 0.3;
	base_trans_front_right_wheel.transform.translation.y = -0.35;
	base_trans_front_right_wheel.transform.translation.z = 0.05;
	base_trans_front_right_wheel.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	geometry_msgs::TransformStamped base_trans_front_left_wheel;
	base_trans_front_left_wheel.header.frame_id = "base_link";
	base_trans_front_left_wheel.child_frame_id = "front_left_wheel";
	base_trans_front_left_wheel.transform.translation.x = 0.3;
	base_trans_front_left_wheel.transform.translation.y = 0.35;
	base_trans_front_left_wheel.transform.translation.z = 0.05;
	base_trans_front_left_wheel.transform.rotation = tf::createQuaternionMsgFromYaw(0);


	geometry_msgs::TransformStamped base_trans_back_right_wheel;
	base_trans_back_right_wheel.header.frame_id = "base_link";
	base_trans_back_right_wheel.child_frame_id = "back_right_wheel";
	base_trans_back_right_wheel.transform.translation.x = -0.3;
	base_trans_back_right_wheel.transform.translation.y = -0.35;
	base_trans_back_right_wheel.transform.translation.z = 0.05;
	base_trans_back_right_wheel.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	geometry_msgs::TransformStamped base_trans_back_left_wheel;
	base_trans_back_left_wheel.header.frame_id = "base_link";
	base_trans_back_left_wheel.child_frame_id = "back_left_wheel";
	base_trans_back_left_wheel.transform.translation.x = -0.3;
	base_trans_back_left_wheel.transform.translation.y = 0.35;
	base_trans_back_left_wheel.transform.translation.z = 0.05;
	base_trans_back_left_wheel.transform.rotation = tf::createQuaternionMsgFromYaw(0);	



	while(ros::ok())
	{

		ros::Time time_now = ros::Time::now();
		odom_trans_base.header.stamp = time_now;
		odom_trans_base.transform.translation.x = cos(angle)*2;
		odom_trans_base.transform.translation.y = sin(angle)*2;
		odom_trans_base.transform.translation.z = .08;
		odom_trans_base.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

		base_trans_front_right_wheel.header.stamp = time_now;
		base_trans_front_left_wheel.header.stamp = time_now;
		base_trans_back_right_wheel.header.stamp = time_now;
		base_trans_back_left_wheel.header.stamp = time_now;

		broadcaster.sendTransform(odom_trans_base);
		broadcaster.sendTransform(base_trans_front_right_wheel);
		broadcaster.sendTransform(base_trans_front_left_wheel);
		broadcaster.sendTransform(base_trans_back_right_wheel);
		broadcaster.sendTransform(base_trans_back_left_wheel);

		angle += degree/4;

		loop_rate.sleep();

	}

	return 0;



}
