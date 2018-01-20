/*
 * Author: 	Abdul Moeed Zafar
 * Date:	29 Nov, 2017
 *
 * File:	planning_node.cpp
 */

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


using namespace ak_planner;

void generateTrajectoryPointsFromPath(GraphStateList solution_path, std::vector<std::vector<double> >& traj_points)
{
	for(int i=0 ; i<solution_path.size() ; i++)
	{
		std::vector<double> traj_single_point;
		traj_single_point = solution_path[i].getStateVariables();

		traj_points.push_back(traj_single_point);

		//TODO: Interpolation between two points using dubins distance.
	}

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "planning_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Duration sleep_time(2);
	sleep_time.sleep();


	std::string environment_type;
	std::cout << "Enter environment setup. FVE-Doc environment (fve) or Maze environment (maze): ";
	std::cin >> environment_type;
	std::cout << std::endl;


	// FVE Obstacle:----------------------------------------------------------------------------	
	std::vector<std::vector<double> > object_primitive_coords_3d;

	std::vector<double> obstacle1;
	obstacle1.resize(6);
	obstacle1[0] = 3;   		//origin x
	obstacle1[1] = 2;   		//origin y
	obstacle1[2] = 0.5;    		//origin z
	obstacle1[3] = 1.5;    		//size x
	obstacle1[4] = 1.5;    		//size y
	obstacle1[5] = 1;      		//size z
	object_primitive_coords_3d.push_back(obstacle1);


	// Maze Obstacles:-----------------------------------------------------------	
	std::vector<std::vector<double> > object_primitive_coords_maze_3d;
	
	std::vector<double> obstacle_maze1;
	obstacle_maze1.resize(6);
	obstacle_maze1[0] = 6.0;      	//origin x
	obstacle_maze1[1] = 4.0; 	    //origin y
	obstacle_maze1[2] = 0.5;    	//origin z
	obstacle_maze1[3] = 1.0;    	//size x
	obstacle_maze1[4] = 6.0;    	//size y
	obstacle_maze1[5] = 1;      	//size z
	object_primitive_coords_maze_3d.push_back(obstacle_maze1);

	std::vector<double> obstacle_maze2;
	obstacle_maze2.resize(6);
	obstacle_maze2[0] = 3.0;      	//origin x
	obstacle_maze2[1] = 7.5; 	    //origin y
	obstacle_maze2[2] = 0.5;    	//origin z
	obstacle_maze2[3] = 7.0;    	//size x
	obstacle_maze2[4] = 1.0;    	//size y
	obstacle_maze2[5] = 1;      	//size z
	object_primitive_coords_maze_3d.push_back(obstacle_maze2);

	std::vector<double> obstacle_maze3;
	obstacle_maze3.resize(6);
	obstacle_maze3[0] = 0.0;      	//origin x
	obstacle_maze3[1] = 3.5; 	    //origin y
	obstacle_maze3[2] = 0.5;    	//origin z
	obstacle_maze3[3] = 1.0;    	//size x
	obstacle_maze3[4] = 7.0;    	//size y
	obstacle_maze3[5] = 1;      	//size z
	object_primitive_coords_maze_3d.push_back(obstacle_maze3);

	std::vector<double> obstacle_maze4;
	obstacle_maze4.resize(6);
	obstacle_maze4[0] = 3.0;      	//origin x
	obstacle_maze4[1] = 2.0; 	    //origin y
	obstacle_maze4[2] = 0.5;    	//origin z
	obstacle_maze4[3] = 1.0;    	//size x
	obstacle_maze4[4] = 5.0;    	//size y
	obstacle_maze4[5] = 1;      	//size z
	object_primitive_coords_maze_3d.push_back(obstacle_maze4);


	// Start & Goal State For FVE Document:-----------------------------------------------------------------------------
	double start_state_x = 0.0; 
	double start_state_y = 2.0;	
	double start_state_theta = (0.0 * (M_PI/180.0));

	double goal_state_x = 6.0; 
	double goal_state_y = 2.0; 
	double goal_state_theta = (0.0 * (M_PI/180.0));

	// Start & Goal State For Maze:----------------------------------------------------------------------------------
	double start_state_maze_x = 1.5; 
	double start_state_maze_y = 0.0;	
	double start_state_maze_theta = (90.0 * (M_PI/180.0)); 

	double goal_state_maze_x = 7.5; 
	double goal_state_maze_y = 7.0; 
	double goal_state_maze_theta = (90.0 * (M_PI/180.0));  
	//--------------------------------------------------------------------------------------------------------------

	double goal_pose_x;
	double goal_pose_y;
	double goal_pose_theta;

	if(environment_type == "fve")
	{
		goal_pose_x = goal_state_x;
		goal_pose_y = goal_state_y;
		goal_pose_theta = goal_state_theta;
	}
	else if(environment_type == "maze")
	{
		goal_pose_x = goal_state_maze_x;
		goal_pose_y = goal_state_maze_y;
		goal_pose_theta = goal_state_maze_theta;
	}
	else
	{
		ROS_ERROR_STREAM("Invalid environment type specified. Cannot set goal_pose_* .");
		throw std::runtime_error("Invalid environment type specified. Cannot set goal_pose_* .");
	}


	// Parameters For The Planner:-----------------------
	double oc_size_x = 10.0;
	double oc_size_y = 10.0; 
	double oc_origin_x = -1.0;
	double oc_origin_y = -1.0;
	double oc_resolution = 0.025;

	double bfs_goal_region_radius = 0.1; 
	double bfs_weight_multiplier = 1.0; 		// For maze: bfs_weight_multiplier = 2.0,   For fve: bfs_weight_multiplier = 1
	double bfs_obstacle_inflation = 0.6; 

	double d_weight_multiplier = 1.0;

	double heuristic_weight_multiplier = 10;

	bool verbosity = true;


	if(environment_type == "fve")
	{
		rover_simulation::RoverSimulator rover_simulator;

		WeightedAStarPlanner wA_planner(oc_size_x, oc_size_y, oc_origin_x, oc_origin_y, oc_resolution, 
		bfs_goal_region_radius, bfs_weight_multiplier, bfs_obstacle_inflation, d_weight_multiplier, object_primitive_coords_3d, 
		heuristic_weight_multiplier);
		
		wA_planner.plan(start_state_x, start_state_y, start_state_theta, goal_state_x, goal_state_y, goal_state_theta, verbosity);

		std::vector<std::vector<double> > path_trajectory;
		generateTrajectoryPointsFromPath(wA_planner.solution_path_, path_trajectory);

		//Interpolating Trajectory
		std::vector<std::vector<double> > interpolated_path_trajectory;
		// interpolated_path_trajectory = path_trajectory;
		wA_planner.getInterpolatedSolutionPath(interpolated_path_trajectory);
		//INTERPOLATED PATH
		std::cout << "interpolated_path_trajectory size: " << interpolated_path_trajectory.size() << std::endl;

		geometry_msgs::PoseArray path_trajectory_pose_array;
		for(int i = 0 ; i < interpolated_path_trajectory.size() ; i++)
		{
			geometry_msgs::Pose rover_pose;
			rover_pose.position.x = interpolated_path_trajectory[i].at(0);
			rover_pose.position.y = interpolated_path_trajectory[i].at(1);
			rover_pose.position.z = 0;
			rover_pose.orientation = tf::createQuaternionMsgFromYaw( interpolated_path_trajectory[i].at(2) );

			path_trajectory_pose_array.poses.push_back(rover_pose);		
		}

		// Simulation:		
		ros::Publisher trajectory_publisher = nh.advertise<geometry_msgs::PoseArray>("/planned_trajectory_topic", 100);
		trajectory_publisher.publish(path_trajectory_pose_array);


		// Error calculation between trajectory final pose and goal pose. 
		std::vector<double> final_pose = interpolated_path_trajectory.back();
		double error_x = goal_pose_x - final_pose[0];
		double error_y = goal_pose_y - final_pose[1];
		double error_theta = (goal_pose_theta - final_pose[2]) * 180.0 / 3.142;
		std::cout << "Error x: " << error_x << "\t\tError y: " << error_y << "\t\tError theta: " << error_theta << std::endl;

		GraphState final_state = wA_planner.expanded_goal_state_;
		final_state.printStateAttributes();

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

				rover_simulator.displayErrorText(error_x, error_y, error_theta, goal_pose_x, goal_pose_y);
				//rover_simulator.publishTrajectory();
			} 
			else
			{
				break;
			}

		}


	}
	else if(environment_type == "maze")
	{
		rover_simulation::RoverSimulator rover_simulator;
		
		WeightedAStarPlanner wA_planner(oc_size_x, oc_size_y, oc_origin_x, oc_origin_y, oc_resolution, 
		bfs_goal_region_radius, bfs_weight_multiplier, bfs_obstacle_inflation, d_weight_multiplier, object_primitive_coords_maze_3d, 
		heuristic_weight_multiplier);

		wA_planner.plan(start_state_maze_x, start_state_maze_y, start_state_maze_theta, goal_state_maze_x, goal_state_maze_y, goal_state_maze_theta, verbosity);

		std::vector<std::vector<double> > path_trajectory;
		generateTrajectoryPointsFromPath(wA_planner.solution_path_, path_trajectory);

		//Interpolating Trajectory
		std::vector<std::vector<double> > interpolated_path_trajectory;
		// interpolated_path_trajectory = path_trajectory;
		wA_planner.getInterpolatedSolutionPath(interpolated_path_trajectory);
		//INTERPOLATED PATH
		std::cout << "interpolated_path_trajectory size: " << interpolated_path_trajectory.size() << std::endl;

		geometry_msgs::PoseArray path_trajectory_pose_array;
		for(int i = 0 ; i < interpolated_path_trajectory.size() ; i++)
		{
			geometry_msgs::Pose rover_pose;
			rover_pose.position.x = interpolated_path_trajectory[i].at(0);
			rover_pose.position.y = interpolated_path_trajectory[i].at(1);
			rover_pose.position.z = 0;
			rover_pose.orientation = tf::createQuaternionMsgFromYaw( interpolated_path_trajectory[i].at(2) );

			path_trajectory_pose_array.poses.push_back(rover_pose);		
		}

		// Simulation:		
		ros::Publisher trajectory_publisher = nh.advertise<geometry_msgs::PoseArray>("/planned_trajectory_topic", 100);
		trajectory_publisher.publish(path_trajectory_pose_array);

		// Error calculation between trajectory final pose and goal pose. 
		std::vector<double> final_pose = interpolated_path_trajectory.back();
		double error_x = goal_pose_x - final_pose[0];
		double error_y = goal_pose_y - final_pose[1];
		double error_theta = (goal_pose_theta - final_pose[2]) * 180.0 / 3.142;
		std::cout << "Error x: " << error_x << "\t\tError y: " << error_y << "\t\tError theta: " << error_theta << std::endl;

		GraphState final_state = wA_planner.expanded_goal_state_;
		final_state.printStateAttributes();


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

				rover_simulator.displayErrorText(error_x, error_y, error_theta, goal_pose_x, goal_pose_y);
				//rover_simulator.publishTrajectory();
			} 
			else
			{
				break;
			}

		}

	}
	else 
	{
		ROS_ERROR_STREAM("Valid environment type not specified. Aborting planning. ");	
	}



	std::cout << "End of Node" << std::endl;

	return 0;
}