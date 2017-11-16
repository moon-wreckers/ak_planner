/*
 * Author: 	Abdul Moeed Zafar
 * Date:	11 Nov, 2017
 *
 * File:	dubins_heuristic.h
 */

#ifndef _DUBINS_HEURISTIC_H_
#define _DUBINS_HEURISTIC_H_

#include <ak_planner/graph_state.h>
#include <ak_planner/dubins_distance.h>
#include <boost/shared_ptr.hpp>
#include <ak_planner/ak_planner_shared_variables.h>


namespace ak_planner
{

	class DubinsHeuristic
	{

	private:

		int weight_multiplier_;
		std::vector<double> goal_pose_;
		bool is_admissible_;

		std::vector<double> goal_region_thres_;

		bool is_goal_set_;


	public:

		DubinsHeuristic(double weight_multiplier);
		~DubinsHeuristic();

		inline void setWeightMultiplier(const int& weight_multiplier);
		inline int getWeightMultipler();
		inline bool isAdmissible();

		double getGoalHeuristic(const GraphState& graph_state);
		void setGoal(const std::vector<double>& goal_pose);

		//For Debugging

	};


	typedef boost::shared_ptr<DubinsHeuristic> DubinsHeuristicPtr;


	inline void DubinsHeuristic::setWeightMultiplier(const int& weight_multiplier)
	{
		assert (weight_multiplier >= 1);
		weight_multiplier_ = weight_multiplier;
	}

	inline int DubinsHeuristic::getWeightMultipler()
	{
		return weight_multiplier_;
	}

	inline bool DubinsHeuristic::isAdmissible()
	{
		return is_admissible_;
	}



} // end namespace ak_planner

#endif