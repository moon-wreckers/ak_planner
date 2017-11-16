/*
 * Author: 	Abdul Moeed Zafar
 * Date:	11 Nov, 2017
 *
 * File:	dubins_heuristic.cpp
 */

#include <ak_planner/dubins_heuristic.h>
#include <ak_planner/dubins_distance.h>
#include <cmath>


namespace ak_planner
{

	DubinsHeuristic::DubinsHeuristic(double weight_multiplier) : is_admissible_(true), is_goal_set_(false)
	{
		setWeightMultiplier(weight_multiplier);
		goal_pose_.resize(3);

		goal_region_thres_.resize(3);
		goal_region_thres_[0] = ak_planner_shared_variables::goal_region_x_threshold;
		goal_region_thres_[1] = ak_planner_shared_variables::goal_region_y_threshold;
		goal_region_thres_[2] = ak_planner_shared_variables::goal_region_theta_threshold;
	}


	DubinsHeuristic::~DubinsHeuristic()
	{

	}


	void DubinsHeuristic::setGoal(const std::vector<double>& goal_pose)
	{
		assert(goal_pose.size() == 3);
		goal_pose_ = goal_pose;
		is_goal_set_ = true;
	}


	double DubinsHeuristic::getGoalHeuristic(const GraphState& graph_state)
	{
		assert(is_goal_set_);

		std::vector<double> state_variables = graph_state.getStateVariables();

		if( (std::abs(state_variables[0] - goal_pose_[0]) < goal_region_thres_[0]) && 
			(std::abs(state_variables[1] - goal_pose_[1]) < goal_region_thres_[1]) &&
			(std::abs(state_variables[2] - goal_pose_[2]) < goal_region_thres_[2]) )
		{
			return 0;
		}
		
		double q0[3];
		q0[0] = state_variables[0];
		q0[1] = state_variables[1];
		q0[2] = state_variables[2];

    	double q1[3];
		q1[0] = goal_pose_[0];
		q1[1] = goal_pose_[1];
		q1[2] = goal_pose_[2];

		dubins_distance::DubinsPath path;
    	dubins_distance::dubins_init( q0, q1, ak_planner_shared_variables::rover_min_turning_radius, &path);
    	// dubins_distance::printDubinsPath(&path, ak_planner_shared_variables::dubins_path_resolution);
    	
    	// dubins_distance::dubins_path_sample_many( &path, printConfiguration, ak_planner_shared_variables::dubins_path_resolution,; NULL);
    	
    	// std::vector<std::vector<double> > dubins_path_vector;
	    // dubins_distance::getDubinsPath(&path, ak_planner_shared_variables::dubins_path_resolution, dubins_path_vector);
	    // dubins_path_vector.push_back(goal_pose_);

    	double path_heur_val = dubins_path_length(&path) / ak_planner_shared_variables::dubins_path_resolution;

    	double multiplied_heur_val = ((float) getWeightMultipler() * path_heur_val );
    	// std::cout << "weight multiplier: " << getWeightMultipler() << std::endl;
    	// std::cout << "path_heur_val: " << multiplied_heur_val << std::endl;
    	assert (multiplied_heur_val >= 0);

		return multiplied_heur_val; // Added 0.5 to round off
	}



} // end namespace ak_planner