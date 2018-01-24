/*
 * Author: 	Abdul Moeed Zafar
 * Date:	24 Jan, 2018
 *
 * File:	planner_client_node.cpp
 */

#include <ros/ros.h>
#include <ak_planner/PlanPath.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "Plan Path Client");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<ak_planner::PlanPath>("plan_path");

	ak_planner::PlanPath srv;
	srv.request.start_pose.x = 0.0;
	srv.request.start_pose.y = 2.0;
	srv.request.start_pose.z = (0.0 * (M_PI/180.0));
	
	srv.request.goal_pose.x = 7.0;
	srv.request.goal_pose.y = 2.0;
	srv.request.goal_pose.z = (335.0 * (M_PI/180.0));

	if(client.call(srv))
	{
		std::cout << "Trajectory size: " << srv.response.trajectory.size() << std::endl;
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
        return 1;
	}

	return 0;
		

}