/*
 * Author: 	Abdul Moeed Zafar
 * Date:	24 Jan, 2018
 *
 * File:	planner_interface.cpp
 */

#include <ak_planner/planner_interface.h>

namespace ak_planner
{

	PlannerInterface::PlannerInterface()
	{
		std::vector<double> obstacle1;
		obstacle1.resize(6);
		obstacle1[0] = 3;   		//origin x
		obstacle1[1] = 2;   		//origin y
		obstacle1[2] = 0.5;    		//origin z
		obstacle1[3] = 1.5;    		//size x
		obstacle1[4] = 1.5;    		//size y
		obstacle1[5] = 1;      		//size z
		obstacles_.push_back(obstacle1);

		start_pose_.resize(3);
		start_pose_[0] = 0.0;
		start_pose_[1] = 2.0;
		start_pose_[2] = (0.0 * (M_PI/180.0));

		goal_pose_.resize(3);
		goal_pose_[0] = 6.0;
		goal_pose_[1] = 2.0;
		goal_pose_[2] = (0.0 * (M_PI/180.0));


		wA_planner_ = boost::make_shared<WeightedAStarPlanner>(WeightedAStarPlanner(parameters_.oc_size_x, 
															   parameters_.oc_size_y,
															   parameters_.oc_origin_x,
															   parameters_.oc_origin_y,
															   parameters_.oc_resolution,
															   parameters_.bfs_goal_region_radius,
															   parameters_.bfs_weight_multiplier,
															   parameters_.bfs_obstacle_inflation,
															   parameters_.dubin_weight_multiplier,
															   obstacles_,
															   parameters_.heuristic_weight_multiplier));

		trajectory_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/planned_trajectory_topic", 100);

		final_pose_error_.resize(3);
	}

	void PlannerInterface::generateTrajectoryPointsFromPath(GraphStateList solution_path)
	{
		for(int i=0 ; i<solution_path.size() ; i++)
		{
			std::vector<double> traj_single_point;
			traj_single_point = solution_path[i].getStateVariables();
			raw_path_trajectory_.push_back(traj_single_point);
		}
	}

	void PlannerInterface::planPath(std::vector<double> start_pose, std::vector<double> goal_pose)
	{
		start_pose_ = start_pose;
		goal_pose_ = goal_pose;
		
		wA_planner_->plan(start_pose_[0], start_pose_[1], start_pose_[2], 
						  goal_pose_[0], goal_pose_[1], goal_pose_[2],
						  parameters_.verbosity);

		generateTrajectoryPointsFromPath(wA_planner_->solution_path_);
		wA_planner_->getInterpolatedSolutionPath(interpolated_path_trajectory_);

		for(int i = 0 ; i < interpolated_path_trajectory_.size() ; i++)
		{
			geometry_msgs::Pose rover_pose;
			rover_pose.position.x = interpolated_path_trajectory_[i].at(0);
			rover_pose.position.y = interpolated_path_trajectory_[i].at(1);
			rover_pose.position.z = 0;
			rover_pose.orientation = tf::createQuaternionMsgFromYaw( interpolated_path_trajectory_[i].at(2) );
			path_pose_array_.poses.push_back(rover_pose);		
		}

		std::vector<double> final_pose = interpolated_path_trajectory_.back();
		final_pose_error_[0] = goal_pose_[0] - final_pose[0];
		final_pose_error_[1] = goal_pose_[1] - final_pose[1];
		final_pose_error_[2] = (goal_pose_[2] - final_pose[2]) * 180.0 / 3.142;
		std::cout << "Error x: " << final_pose_error_[0] << "\t\tError y: " << final_pose_error_[1] << "\t\tError theta: " << final_pose_error_[2] << std::endl;

	}


	void PlannerInterface::runSimulation()
	{
		trajectory_publisher_.publish(path_pose_array_);

		while(ros::ok())
		{
			char command;
			std::cout << "Enter Y to play trajectory. Press any key to finish.  ";
			std::cin >> command;

			if(command == 'Y' || command == 'y')
			{
				ros::spinOnce();
				rover_simulator_.runSimulation();
				rover_simulator_.displayErrorText(final_pose_error_[0], final_pose_error_[1], final_pose_error_[2], goal_pose_[0], goal_pose_[1]);
			} 
			else
			{
				break;
			}

		}
	}


	void PlannerInterface::getInterpolatedPathTrajectory(std::vector<std::vector<double> >& final_path)
	{
		for(int i=0 ; i<interpolated_path_trajectory_.size() ; i++)
		{
			final_path.push_back(interpolated_path_trajectory_[i]);
		}
	}


}