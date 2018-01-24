/*
 * Author: 	Abdul Moeed Zafar
 * Date:	24 Jan, 2018
 *
 * File:	planner_interface.h
 */

#ifndef _PLANNER_INTERFACE_
#define _PLANNER_INTERFACE_


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <ak_planner/moveit_interface.h>
#include <ak_planner/ak_planner_shared_variables.h>

#include <ak_planner/bfs_2d_heuristic.h>
#include <ak_planner/occupancy_grid_2d.h>
#include <ak_planner/graph_state_manager.h>
#include <ak_planner/graph_state.h> 
#include <ak_planner/dubins_heuristic.h>

#include <ak_planner/weighted_A_star_planner.h>

#include <algorithm>



#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <ak_planner/dubins_distance.h>

#include <ak_simulation/rover_simulator.h>


#include <ros/package.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <ak_planner/motion_primitives_parser.h>

namespace ak_planner
{

	class PlannerInterface
	{
		
		struct PlannerParameters
		{

			double oc_size_x;
			double oc_size_y;
			double oc_origin_x;
			double oc_origin_y;
			double oc_resolution;

			double bfs_goal_region_radius;
			double bfs_weight_multiplier;
			double bfs_obstacle_inflation;
			double dubin_weight_multiplier;
			double heuristic_weight_multiplier;

			bool verbosity;

			PlannerParameters() : oc_size_x(10.0), 
								  oc_size_y(10.0),
								  oc_origin_x(-1.0),
								  oc_origin_y(-1.0),
								  oc_resolution(0.15),
								  bfs_goal_region_radius(0.1),
								  bfs_weight_multiplier(1.0),
								  bfs_obstacle_inflation(0.8),
								  dubin_weight_multiplier(1.0),
								  heuristic_weight_multiplier(10),
								  verbosity(true)
			{

			}
		};


	private:

		ros::NodeHandle nh_;

		std::vector<std::vector<double> > obstacles_;
		std::vector<double> start_pose_;
		std::vector<double> goal_pose_;
		
		PlannerParameters parameters_;

		WeightedAStarPlannerPtr wA_planner_;

		std::vector<std::vector<double> > raw_path_trajectory_;
		std::vector<std::vector<double> > interpolated_path_trajectory_;
		geometry_msgs::PoseArray path_pose_array_;

		//Simulation
		ros::Publisher trajectory_publisher_; // = nh.advertise<geometry_msgs::PoseArray>("/planned_trajectory_topic", 100);
		rover_simulation::RoverSimulator rover_simulator_;
		std::vector<double> final_pose_error_;

		void generateTrajectoryPointsFromPath(GraphStateList solution_path);

	public:

		PlannerInterface();
		~PlannerInterface() {}

		void planPath(std::vector<double> start_pose, std::vector<double> goal_pose);
		void runSimulation();

		void getInterpolatedPathTrajectory(std::vector<std::vector<double> >& final_path);

	};


} //end namespace ak_planner

#endif