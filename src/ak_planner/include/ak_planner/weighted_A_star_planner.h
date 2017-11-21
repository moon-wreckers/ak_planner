/*
 * Author: 	Abdul Moeed Zafar
 * Date:	13 Nov, 2017
 *
 * File:	weighted_A_star_planner.h
 */

#ifndef _WEIGHTED_A_STAR_PLANNER_H_
#define _WEIGHTED_A_STAR_PLANNER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <time.h>
#include <algorithm>

#include <ak_planner/ak_planner_shared_variables.h>
#include <ak_planner/graph_state.h>
#include <ak_planner/graph_state_manager.h>
#include <ak_planner/moveit_interface.h>
#include <ak_planner/bfs_2d_heuristic.h>
#include <ak_planner/dubins_heuristic.h>
#include <ak_planner/occupancy_grid_2d.h>
#include <ak_planner/motion_primitives.h>

#include <ak_planner/motion_primitives_parser.h>



namespace ak_planner
{

	class WeightedAStarPlanner
	{

	public:

		GraphState expanded_goal_state_;
		
		bool is_goal_state_expanded_;		
		double cost_prim_;							//Think about this maybe? Initializing it to some value right now.
		double heuristic_weight_multiplier_;

		GraphStateList open_queue_;
		GraphStateList closed_list_;
		GraphStateList solution_path_;


		MoveitInterfacePtr moveit_interface_;
		OccupancyGrid2DPtr occupancy_grid_2d_;
		BFS2DHeuristicPtr bfs_2d_heuristic_;
		DubinsHeuristicPtr dubins_heuristic_;
		GraphStateManagerPtr graph_state_manager_;
		
		MotionPrimitivesPtr motion_primitives_; 	// InshaAllah will be removed. Will replace this by motion_primitives_parser_ below.
		
		MotionPrimitivesParserPtr motion_primitives_parser_;

		bool verbosity_;



		WeightedAStarPlanner(const double oc_size_x, 
							 const double oc_size_y, 
							 const double oc_origin_x, 
							 const double oc_origin_y, 
							 const double oc_resolution,  

							 const double bfs_goal_region_radius, 
							 const double bfs_weight_multiplier, 
							 const double bfs_obstacle_inflation, 

							 const double d_weight_multiplier, 

							 const std::vector<std::vector<double> > object_primitive_coords_3d, 
							 const double heuristic_weight_multiplier);
		
		~WeightedAStarPlanner();

		int getDiscreteAngle(double angle);

		void getDiscretePose(double x_cont, double y_cont, double theta_cont, int& x_disc, int& y_disc, int& theta_disc);

		void insertStateInOpenQueue(GraphState& graph_state);

		void initPlanner(const double start_state_x, const double start_state_y, const double start_state_theta, 
			const double goal_state_x, const double goal_state_y, const double goal_state_theta);
		
		GraphState popMinFValStateFromQueue();

		bool isStateClosed(GraphState& graph_state);

		bool isPresentInOpen(GraphState& graph_state);

		GraphState getSimilarStateFromOpen(GraphState graph_state);

		bool isGoalConditionSatisfied(GraphState graph_state);

		double getHeuristicValue(GraphState graph_state);

		std::vector<GraphState> getSuccessors(GraphState graph_state);

		bool isInEnvironmentBounds(GraphState& graph_state);

		void expandGraphState(GraphState current_graph_state);

		void getSolutionPath();

		void getInterpolatedSolutionPath(std::vector<std::vector<double> >& interpolated_path);

		void printSolutionPathStates();

		bool plan(const double start_state_x, const double start_state_y, const double start_state_theta, 
			const double goal_state_x, const double goal_state_y, const double goal_state_theta, bool verbosity);


	};


	typedef boost::shared_ptr<WeightedAStarPlanner> WeightedAStarPlannerPtr;


} // end namespace ak_planner


#endif
