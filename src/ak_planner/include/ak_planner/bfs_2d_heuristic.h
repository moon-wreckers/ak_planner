/*
 * Author: 	Abdul Moeed Zafar
 * Date:	8 Nov, 2017
 *
 * File:	bfs_2d_heuristic.h
 */

#ifndef _BFS_2D_HEURISTIC_H_
#define _BFS_2D_HEURISTIC_H_

#include <ak_planner/graph_state.h>
#include <ak_planner/occupancy_grid_2d.h>
#include <ak_planner/BFS_2D.h>


namespace ak_planner
{

	class BFS2DHeuristic
	{

	private:

		int weight_multiplier_;
		std::vector<double> goal_pose_;
		bool is_admissible_;

		double goal_projection_x_;
		double goal_projection_y_;
		double goal_region_radius_;
		double obstacle_inflation_;

		BFS_2DPtr bfs_;
		OccupancyGrid2DPtr occupancy_grid_2d_;


	public:

		BFS2DHeuristic(double goal_region_radius, int weight_multiplier, OccupancyGrid2DPtr occupancy_grid_2d, double obstacle_inflation);
		~BFS2DHeuristic();

		inline void setWeightMultiplier(const int& weight_multiplier);
		inline int getWeightMultipler();
		inline bool isAdmissible();

		double getGoalHeuristic(const GraphState graph_state);
		void setGoal(const std::vector<double>& goal_pose);
		void updateBFS2DGrid();



		// Testing Purposes:
		BFS_2DPtr getBFS_2DPtr()
		{
			return bfs_;
		}

		void printBFS2DHeuristicMap();


	};

	
	typedef boost::shared_ptr<BFS2DHeuristic> BFS2DHeuristicPtr;


	inline void BFS2DHeuristic::setWeightMultiplier(const int& weight_multiplier)
	{
		assert (weight_multiplier >= 1);
		weight_multiplier_ = weight_multiplier;
	}

	inline int BFS2DHeuristic::getWeightMultipler()
	{
		return weight_multiplier_;
	}

	inline bool BFS2DHeuristic::isAdmissible()
	{
		return is_admissible_;
	}


} // end namespace ak_planner



#endif 