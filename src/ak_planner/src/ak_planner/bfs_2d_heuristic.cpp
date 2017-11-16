/*
 * Author: 	Abdul Moeed Zafar
 * Date:	8 Nov, 2017
 *
 * File:	bfs_2d_heuristic.cpp
 */

#include <ak_planner/bfs_2d_heuristic.h>

namespace ak_planner
{

	BFS2DHeuristic::BFS2DHeuristic(double goal_region_radius, int weight_multiplier, OccupancyGrid2DPtr occupancy_grid_2d, 
		double obstacle_inflation) : is_admissible_(true), goal_region_radius_(goal_region_radius), occupancy_grid_2d_(occupancy_grid_2d), 
			obstacle_inflation_(obstacle_inflation)
	{
		setWeightMultiplier(weight_multiplier);
		
		int dim_x, dim_y;
		occupancy_grid_2d_->getGridSize(dim_x, dim_y);

		bfs_.reset(new BFS_2D(dim_x, dim_y));

		goal_pose_.resize(3);
	}


	BFS2DHeuristic::~BFS2DHeuristic()
	{

	}

	void BFS2DHeuristic::setGoal(const std::vector<double>& goal_pose)
	{
		assert(goal_pose.size() == 3);

		goal_pose_ = goal_pose;
		goal_projection_x_ = goal_pose_[0];
		goal_projection_y_ = goal_pose_[1];
		
		int gx, gy;
		occupancy_grid_2d_->worldToGrid(goal_projection_x_, goal_projection_y_, gx, gy);
		std::cout << "Setting Goal - gx: " << gx << " gy: " << gy << std::endl;
		if(bfs_->isWall(gx, gy))
		{
			ROS_ERROR_STREAM("Goal position ground projection occupied by obstacle!");
            throw std::runtime_error("Goal position ground projection occupied by obstacle!");
		}

		bfs_->run(gx, gy);
	}

	void BFS2DHeuristic::updateBFS2DGrid()
	{
		int dim_x, dim_y;
		occupancy_grid_2d_->getGridSize(dim_x, dim_y);
		std::cout << "In updateBFS2DGrid - dim_x: " << dim_x << " dim_y: " << dim_y << std::endl;
		int walls = 0;
		
		for(int y = 0 ; y < dim_y ; y++)
		{
			for(int x = 0 ; x < dim_x ; x++)
			{
				double distance = occupancy_grid_2d_->getDistanceFromGridCoordinates(x, y);

				// double wx, wy;
				// occupancy_grid_2d_->gridToWorld(x, y, wx, wy);
				// std::cout << "x: " << x << " y: " << y << "\twx: " << wx << " wy: " << wy << std::endl;

				if(distance <= obstacle_inflation_)
				{
					bfs_->setWall(x, y); // Changed from (x+1, y+1) to (x, y) 
					walls++;

					// double wx, wy;
					// occupancy_grid_2d_->gridToWorld(x, y, wx, wy);
					// std::cout << "Obstacle: wx: " << wx << " wy: " << wy << "   x: " << x << " y: " << y << std::endl;
				}
				//std::cout << bfs_->getDistance(x+1, y+1) << "\t";
			}
			//std::cout << std::endl;
		}

		std::cout << "Updated BFS2D grid with obstacles" << std::endl;
	}


	double BFS2DHeuristic::getGoalHeuristic(const GraphState graph_state) //TODO
	{
		std::vector<double> state_variables = graph_state.getStateVariables(); // TODO: replace with getStateVariablesDiscrete

		int x, y;
		occupancy_grid_2d_->worldToGrid(state_variables[0], state_variables[1], x, y); 
		double heur_val = bfs_->getDistance(x, y); //TODO: remove worldToGrid thing above and just use graph_state's x_coord_ and y_coord_
		// std::cout << "heur val inside getGoalHeuristic: " << heur_val << std::endl;
		// std::cout << "heur val after multiplication: " << ((double)heur_val * occupancy_grid_2d_->getResolution()) << std::endl;
		// std::cout << "goal_region_radius_: " << goal_region_radius_ << std::endl;
		// If goal is within threshold region, return 0
        if (((double)heur_val * occupancy_grid_2d_->getResolution()) <= goal_region_radius_)
        {
        	// std::cout << "Inside goal region" << std::endl;
            return 0;
            //cost = 0;
        }

        // std::cout << "heur val2 inside getGoalHeuristic: " << heur_val << std::endl;
		
		double multiplied_heur_val = ((double) getWeightMultipler()) * heur_val;

		// std::cout << "multiplied heur val inside getGoalHeuristic: " << multiplied_heur_val << std::endl;

		assert (multiplied_heur_val >= 0);

		return multiplied_heur_val;
	}


	void BFS2DHeuristic::printBFS2DHeuristicMap()
	{
		std::cout << "Printing BFS Map: " << std::endl << std::endl;

		double dim_x, dim_y;
		occupancy_grid_2d_->getWorldSize(dim_x, dim_y);

		int grid_x , grid_y;
		occupancy_grid_2d_->getGridSize(grid_x, grid_y);		

		std::cout << "grid_x: " << grid_x << std::endl;
		std::cout << "grid_y: " << grid_y << std::endl;

		std::cout << "occupancy_grid_2d_->origin_x_: " << occupancy_grid_2d_->origin_x_ << std::endl;
		std::cout << "dim_x+occupancy_grid_2d_->origin_x_: " << dim_x+occupancy_grid_2d_->origin_x_ << std::endl;

		std::cout << "occupancy_grid_2d_->origin_y_: " << occupancy_grid_2d_->origin_y_ << std::endl;
		std::cout << "dim_y+occupancy_grid_2d_->origin_y_: " << dim_y+occupancy_grid_2d_->origin_y_ << std::endl;
		

		for(double x = 0 ; x < 2.5 ; x += occupancy_grid_2d_->getResolution())
		{
			for(double y = 0 ; y < 2.5 ; y += occupancy_grid_2d_->getResolution())
			{
				int gx, gy;
				occupancy_grid_2d_->worldToGrid(x, y, gx, gy);
				double distance = bfs_->getDistance(gx, gy);
				if(distance > 10000)
					distance = 10000;
				std::cout << distance << "\t";
				//std::cout << bfs_->getDistance(x+1, y+1) << "\t";
			}
			
			std::cout << std::endl;
			std::cout << std::endl;
		}

		std::cout << std::endl << "Done." << std::endl;

	}





} // end namespace ak_planner