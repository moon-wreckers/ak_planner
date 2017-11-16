/*
 * Author: 	Abdul Moeed Zafar
 * Date:	28 Oct, 2017
 *
 * File:	rover_simulation_test_node.cpp
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <robot_publisher/rover_simulator.h>
#include <robot_publisher/dubins_distance.h>


// For Dubins path printing
int printConfiguration(double q[3], double x, void* user_data) 
{    
    std::cout << q[0] << ", " << q[1] << ", " << q[2] << ", " << x << std::endl;
    // printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
    return 0;
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "rover_simulation_test_node");
	ros::NodeHandle n;


	//---------------------------------------- Testing Dubins Distance -------------------------------------------------------------------


	double q0[] = { 2, 1,  (30 * M_PI/180) };

    double q1[] = { 2.4562, 1.6098, (40.0 * M_PI/180.0) };
    
    double turning_radius = 0.5;
    dubins_distance::DubinsPath path;
    dubins_distance::dubins_init( q0, q1, turning_radius, &path);
    dubins_distance::dubins_path_sample_many( &path, printConfiguration, 0.05, NULL);
    double path_cost = dubins_path_length(&path);
    std::cout << "Path length: " << path_cost << std::endl;

    std::vector<std::vector<double> > dubins_path_vector;
    dubins_distance::getDubinsPath(&path, 0.1, dubins_path_vector);
    std::vector<double> goal_state;
    goal_state.resize(3);
    goal_state[0] = q1[0];
    goal_state[1] = q1[1];
    goal_state[2] = q1[2];
    dubins_path_vector.push_back(goal_state);

    std::cout << "dubins_path_vector size: " << dubins_path_vector.size() << std::endl;

    geometry_msgs::PoseArray trajectory_dubins_path;
	for(int i = 0 ; i < dubins_path_vector.size() ; i++)
	{
		geometry_msgs::Pose rover_pose;
		rover_pose.position.x = dubins_path_vector[i].at(0);
		rover_pose.position.y = dubins_path_vector[i].at(1);
		rover_pose.position.z = 0;
		rover_pose.orientation = tf::createQuaternionMsgFromYaw( dubins_path_vector[i].at(2) );

		trajectory_dubins_path.poses.push_back(rover_pose);		
	}


	//---------------------------------------- Object Visualization Markers Testing ------------------------------------------------------

	// ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
	// uint32_t shape = visualization_msgs::Marker::CUBE;

	// visualization_msgs::Marker marker;
	// marker.header.frame_id = "odom_combined";
	// marker.ns = "basic_shapes";
	// marker.id = 0;
	// marker.type = shape;
	// marker.action = visualization_msgs::Marker::ADD;

	// marker.pose.position.x = 1;
	// marker.pose.position.y = 0;
	// marker.pose.position.z = 0;
	// marker.pose.orientation.x = 0.0;
	// marker.pose.orientation.y = 0.0;
	// marker.pose.orientation.z = 0.0;
	// marker.pose.orientation.w = 1.0;
	
	// marker.scale.x = 0.5;
	// marker.scale.y = 0.5;
	// marker.scale.z = 0.5;

	// marker.color.r = 0.0f;
	// marker.color.g = 1.0f;
	// marker.color.b = 0.0f;
	// marker.color.a = 1.0f;

	// marker.lifetime = ros::Duration();

	// while(marker_pub.getNumSubscribers() < 1)
	// {
	// 	if(!ros::ok())
	// 	{
	// 		return 0;
	// 	}

	// 	ROS_WARN_ONCE("Please create a subscriber to the marker");
	// 	sleep(1);
	// }

	// marker_pub.publish(marker);


	//---------------------------------------- Without Object Visualization Markers in Environment ---------------------------------------

	tf::TransformBroadcaster tf_broadcaster;

	rover_simulation::RoverSimulator rover_simulator;
	ros::Publisher trajectory_publisher = n.advertise<geometry_msgs::PoseArray>("/planned_trajectory_topic", 100);

	float res = 0.01;
	geometry_msgs::PoseArray trajectory1;
	for(int i = 0 ; i < 100 ; i++)
	{
		geometry_msgs::Pose rover_pose;
		rover_pose.position.x = i*res;
		rover_pose.position.y = 0;
		rover_pose.position.z = 0;
		rover_pose.orientation = tf::createQuaternionMsgFromYaw(0);

		trajectory1.poses.push_back(rover_pose);		
	}

	// trajectory_publisher.publish(trajectory1);
	trajectory_publisher.publish(trajectory_dubins_path); // Publishing dubins_path_trajectory
	


	while(ros::ok())
	{
		char command;
		std::cout << "Enter Y to play trajectory. Press any key to finish.  ";
		std::cin >> command;

		if(command == 'Y' || command == 'y')
		{
			// trajectory_publisher.publish(trajectory1);
			ros::spinOnce();
			rover_simulator.runSimulation();
			//rover_simulator.publishTrajectory();
		} 
		else
		{
			break;
		}

	}

	//-----------------------------------------------------------------------------------------------------------------------------------

	return 0;

}