/*
 * Author: 	Abdul Moeed Zafar
 * Date:	1 Nov, 2017
 *
 * File:	ak_planner_shared_variables.h
 */

#ifndef _AK_PLANNER_SHARED_VARIABLES_H_
#define _AK_PLANNER_SHARED_VARIABLES_H_

#include <math.h>
#include <utility>

namespace ak_planner_shared_variables
{

	struct RoverTrajectoryNode
	{

		double x;
		double y;
		double theta;

	};

	
	const std::string moveit_planning_scene_topic = "/planning_scene";
	const std::string moveit_robot_state_topic = "/robot_states";

	const std::string rover_urdf_param_name = "robot_description";
	const std::string world_frame_id = "odom_combined";

	const std::string moveit_rover_x_variable_name = "virtual_joint/x";
	const std::string moveit_rover_y_variable_name = "virtual_joint/y";
	const std::string moveit_rover_theta_variable_name = "virtual_joint/theta";


	const double bfs2d_goal_region_radius = 0.3;


	const double rover_min_turning_radius = 0.99; //0.3
	const double dubins_path_resolution = 0.025;

	const double goal_region_x_threshold = 0.02; //0.2
	const double goal_region_y_threshold = 0.02; //0.2
	const double goal_region_theta_threshold = ((double) M_PI/180 * 5);


	const int NUM_OF_DIRECTIONS = 16;
	
	// double degToRad(double degree)
	// {
	// 	return ((double)(M_PI / 180 * degree)); 
	// }


	// std::pair<double, double> goal_x_limits(-0.1, +0.1);
	// std::pair<double, double> goal_y_limits(-0.1, +0.1);
	// std::pair<double, double> goal_theta_limits(-1*degToRad(10), -1*degToRad(10));
	// std::vector<std::pair<double, double> > dubins_goal_region_limits;
	// dubins_goal_region_limits.push_back(goal_x_limits);
	// dubins_goal_region_limits.push_back(goal_y_limits);
	// dubins_goal_region_limits.push_back(goal_theta_limits);




} //end namespace ak_planner_shared_variables

#endif