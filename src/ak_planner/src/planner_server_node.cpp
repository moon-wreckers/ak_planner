/*
 * Author: 	Abdul Moeed Zafar
 * Date:	24 Jan, 2018
 *
 * File:	planner_server_node.cpp
 */

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ak_planner/planner_interface.h>
#include <ak_planner/PlanPath.h>

using namespace ak_planner;

bool planPath(ak_planner::PlanPath::Request &req, ak_planner::PlanPath::Response &res)
{
	std::vector<double> start_pose;
	start_pose.resize(3);
	start_pose[0] = req.start_pose.x;
	start_pose[1] = req.start_pose.y;
	start_pose[2] = req.start_pose.z;

	std::vector<double> goal_pose;
	goal_pose.resize(3);
	goal_pose[0] = req.goal_pose.x;
	goal_pose[1] = req.goal_pose.y;
	goal_pose[2] = req.goal_pose.z;

	std::vector<std::vector<double> > path;

	PlannerInterface planner_interface;
	planner_interface.planPath(start_pose, goal_pose);
	planner_interface.getInterpolatedPathTrajectory(path);

	// Testing
	// path.clear();
	// path.resize(3);

	// std::vector<double> way_point1;
	// way_point1.resize(3);
	// way_point1[0] = 0.1667;
	// way_point1[1] = 3.0;
	// way_point1[2] = 0.0;
	// path.push_back(way_point1);

	// std::vector<double> way_point2;
	// way_point2.resize(3);
	// way_point2[0] = 0.3333;
	// way_point2[1] = 3.0;
	// way_point2[2] = 0.0;
	// path.push_back(way_point2);
	
	// std::vector<double> way_point3;
	// way_point3.resize(3);
	// way_point3[0] = 0.5;
	// way_point3[1] = 3.0;
	// way_point3[2] = 0.0;
	// path.push_back(way_point3);
	 

	for(int i=0 ; i<path.size() ; i++)
	{
		geometry_msgs::Vector3 path_node;
		path_node.x = path[i].at(0);
		path_node.y = path[i].at(1);
		path_node.z = path[i].at(2);

		res.trajectory.push_back(path_node);
	}

	planner_interface.runSimulation();
	return true;
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "Plan_Path_Server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("plan_path", planPath);
    ROS_INFO("Ready to plan a path");
    ros::spin();

    return 0;
}