/*
 * Author: 	Abdul Moeed Zafar
 * Date:	13 Nov, 2017
 *
 * File:	weighted_A_star_planner.cpp
 */

#include <ak_planner/weighted_A_star_planner.h>

//TODOs: 1) validate obstacle inflation, (DONE)
//		 2) update occupancy_grid_->addObjectToGrid() to read object spec definitions from shared_variables file

struct CompareByFValue 
{
	bool operator()(const ak_planner::GraphState& a, const ak_planner::GraphState& b)
	{
		return a.getFVal() < b.getFVal();
	}
} compareByFVal;


namespace ak_planner
{


	WeightedAStarPlanner::WeightedAStarPlanner(
		const double oc_size_x, 
		const double oc_size_y, 
		const double oc_origin_x, 
		const double oc_origin_y, 
		const double oc_resolution, 

		const double bfs_goal_region_radius, 
		const double bfs_weight_multiplier, 
		const double bfs_obstacle_inflation, 

		const double d_weight_multiplier, 

		const std::vector<std::vector<double> > object_primitive_coords_3d, 
		const double heuristic_weight_multiplier)
	{
		// Extracting object projections in 2D ground plane.
		std::vector<std::vector<double> > object_primitive_coords_2d;
		for(int i = 0 ; i < object_primitive_coords_3d.size() ; i++)
		{
			std::vector<double> object_coords;
			object_coords.resize(4);
			object_coords[0] = object_primitive_coords_3d[i][0];
			object_coords[1] = object_primitive_coords_3d[i][1];
			object_coords[2] = object_primitive_coords_3d[i][3];
			object_coords[3] = object_primitive_coords_3d[i][4];
			
			object_primitive_coords_2d.push_back(object_coords);
		}

		// Instantiating moveit_interface_ and populating obstacles.
		moveit_interface_ = boost::make_shared<MoveitInterface>();
		for(int i = 0 ; i < object_primitive_coords_3d.size() ; i++)
		{
			assert(object_primitive_coords_3d[i].size() == 6);

			moveit_interface_->addEnvObjectToPlanningScene(object_primitive_coords_3d[i][0], object_primitive_coords_3d[i][1], object_primitive_coords_3d[i][2], 
				object_primitive_coords_3d[i][3], object_primitive_coords_3d[i][4], object_primitive_coords_3d[i][5]);
		}

		// Instantiating occupancy_grid_2d_ and adding obstacles in it.
		occupancy_grid_2d_ = boost::make_shared<OccupancyGrid2D>(oc_size_x, oc_size_y, oc_origin_x, oc_origin_y, oc_resolution);
		occupancy_grid_2d_->addObjectToGrid(object_primitive_coords_2d);

		// Instantiating bfs_2d_heuristic_ and adding obstacle regions in it.
		bfs_2d_heuristic_ = boost::make_shared<BFS2DHeuristic>(bfs_goal_region_radius, bfs_weight_multiplier, occupancy_grid_2d_, bfs_obstacle_inflation);
		bfs_2d_heuristic_->updateBFS2DGrid();

		// Instantiating dubins_heuristic_.
		dubins_heuristic_ = boost::make_shared<DubinsHeuristic>(d_weight_multiplier);
		
		// Instantiating graph_state_manager_.
		graph_state_manager_ = boost::make_shared<GraphStateManager>();

		is_goal_state_expanded_ = false;
		cost_prim_ = 1;
		heuristic_weight_multiplier_ = heuristic_weight_multiplier;

		//Instantiating motion_primitives_
		motion_primitives_ = boost::make_shared<MotionPrimitives>();

		//Instantiating motion_primitives_parser_
		motion_primitives_parser_ = boost::make_shared<MotionPrimitivesParser>();
		motion_primitives_parser_->readPrimitives();

		assert(motion_primitives_parser_->prim_resolution_ == oc_resolution);

		verbosity_ = false;
	}	


	WeightedAStarPlanner::~WeightedAStarPlanner()
	{

	}


	int WeightedAStarPlanner::getDiscreteAngle(double angle)
	{
		// int NUMOFDIRS = ak_planner_shared_variables::NUM_OF_DIRECTIONS;
	 //    /* returns the direction index with respect to the PrimArray */
	 //    /* normalize bw 0 to 2pi */
	 //    if (angle < 0.0) {
	 //        angle += 2 * M_PI;
	 //    }
	 //    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
	 //    if (dir == NUMOFDIRS) {
	 //        dir = 0;
	 //    }
	 //    return dir;

		//Starting edit for incorporating Parser

		int angle_id = motion_primitives_parser_->discretizeAngle(angle);

		if(angle_id < 0)
		{
			std::cout << "Error here. angle_id: " << angle_id << std::endl;
			throw 0;
		}

		return angle_id;
	}


	void WeightedAStarPlanner::getDiscretePose(double x_cont, double y_cont, double theta_cont, int& x_disc, int& y_disc, int& theta_disc)
	{
		occupancy_grid_2d_->worldToGrid(x_cont, y_cont, x_disc, y_disc);
		theta_disc = getDiscreteAngle(theta_cont);
	}


	void WeightedAStarPlanner::insertStateInOpenQueue(GraphState& graph_state)
	{
		assert( graph_state.allParamsInit() );

		//int size_queue = (int)open_queue_.size(); // TODO: Dont need this probably. Remove it.

		GraphStateList::iterator it;
		for(it = open_queue_.begin() ; it != open_queue_.end() ; it++)
		{
			if(it->getId() == graph_state.getId())
			{
				*it = graph_state;
				std::sort(open_queue_.begin(), open_queue_.end(), compareByFVal);
				break;
			}
		}

		if(it == open_queue_.end())
		{
			open_queue_.push_back(graph_state);
			std::sort(open_queue_.begin(), open_queue_.end(), compareByFVal);
		}
	}


	void WeightedAStarPlanner::initPlanner( 
		const double start_state_x, 
		const double start_state_y, 
		const double start_state_theta, 

		const double goal_state_x, 
		const double goal_state_y, 
		const double goal_state_theta)
	{


		std::cout << std::endl << "Initializing Planner." << std::endl;

		if(moveit_interface_->isStateInCollision(start_state_x, start_state_y, start_state_theta))
		{
			ROS_ERROR_STREAM("Invalid start state for the robot. Start state in collision. Aborting - initPlanner");
			throw std::runtime_error("Invalid start state for the robot. Start state in collision. Aborting - initPlanner");
		}

		if(moveit_interface_->isStateInCollision(goal_state_x, goal_state_y, goal_state_theta))
		{
			ROS_ERROR_STREAM("Invalid goal state for the robot. Goal state in collision. Aborting - initPlanner");
			throw std::runtime_error("Invalid goal state for the robot. Goal state in collision. Aborting - initPlanner");
		}


		GraphState start_state;		
		start_state.setPrimitiveId(-1);
		start_state.setPredId(-1);
		start_state.setStateType(GraphState::SINGLE_POINT);
		start_state.setStateVariables(start_state_x, start_state_y, start_state_theta);
		start_state.setGVal(0.0);

		int start_state_x_dis, start_state_y_dis, start_state_theta_dis;
		getDiscretePose(start_state_x, start_state_y, start_state_theta, start_state_x_dis, start_state_y_dis, start_state_theta_dis);
		start_state.setStateVariablesDiscrete(start_state_x_dis, start_state_y_dis, start_state_theta_dis);
		

		GraphState goal_state;
		goal_state.setPrimitiveId(-1);
		goal_state.setPredId(-1);
		goal_state.setStateType(GraphState::SINGLE_POINT);
		goal_state.setStateVariables(goal_state_x, goal_state_y, goal_state_theta);
		
		int goal_state_x_dis, goal_state_y_dis, goal_state_theta_dis;
		getDiscretePose(goal_state_x, goal_state_y, goal_state_theta, goal_state_x_dis, goal_state_y_dis, goal_state_theta_dis);
		goal_state.setStateVariablesDiscrete(goal_state_x_dis, goal_state_y_dis, goal_state_theta_dis);
		

		moveit_interface_->displayRobotStartState(start_state_x, start_state_y, start_state_theta);
		moveit_interface_->displayRobotGoalState(goal_state_x, goal_state_y, goal_state_theta);
		

		graph_state_manager_->addStartState(start_state);
		graph_state_manager_->addGoalState(goal_state);


		std::vector<double> goal_state_variables = goal_state.getStateVariables();
		dubins_heuristic_->setGoal(goal_state_variables);
		bfs_2d_heuristic_->setGoal(goal_state_variables);

		// TODO confirm that start_state ID is set here after it is added through graph_state_manager cuz pass by reference. It should be!
		// if(start_state.getId() == GraphStateManager::START_STATE_ID)
		// {
		// 	std::cout << "Confirmed that start state ID gets updated here. Remove the code below and just directly add start state to open queue" << std::endl;
		// 	throw 0;
		// }


		// GraphState start_state_new = graph_state_manager_->getGraphState(GraphStateManager::START_STATE_ID);
		insertStateInOpenQueue(start_state);

		std::cout << "Start State: " << std::endl;
		start_state.printStateAttributes();
		std::cout << std::endl;

		std::cout << "Goal State: " << std::endl;
		goal_state.printStateAttributes();
		std::cout << std::endl;
		std::cout << "-------------------------------------------------------" << std::endl;
	}


	GraphState WeightedAStarPlanner::popMinFValStateFromQueue()
	{
		std::sort(open_queue_.begin(), open_queue_.end(), compareByFVal);

		GraphState graph_state = open_queue_.at(0);
		open_queue_.erase(open_queue_.begin());

		return graph_state;
	}


	bool WeightedAStarPlanner::isStateClosed(GraphState& graph_state)
	{
		bool is_closed = false;
		
		GraphStateList::iterator it;
		for(it = closed_list_.begin() ; it != closed_list_.end() ; it++)
		{
			if(graph_state.getId() == it->getId())
			{
				is_closed = true;

				int it_x, it_y, it_theta;
				it->getStateVariablesDiscrete(it_x, it_y, it_theta);

				int graph_state_x, graph_state_y, graph_state_theta;
				graph_state.getStateVariablesDiscrete(graph_state_x, graph_state_y, graph_state_theta);

				if( (it_x != graph_state_x) || (it_y != graph_state_y) || (it_theta != graph_state_theta) )
				{
					ROS_ERROR_STREAM("States not same are considered closed - isStateClosed");
					throw std::runtime_error("States not same are considered closed - isStateClosed");
				}

				break;
			}
		}

		return is_closed;
	}


	bool WeightedAStarPlanner::isPresentInOpen(GraphState& graph_state)
	{
		bool is_present = false;
		
		GraphStateList::iterator it;
		for(it = open_queue_.begin() ; it != open_queue_.end() ; it++)
		{
			if(graph_state.getId() == it->getId())
			{
				is_present = true;
				break;
			}
		}

		return is_present;
	}


	GraphState WeightedAStarPlanner::getSimilarStateFromOpen(GraphState graph_state)
	{
		GraphState graph_state_in_open;

		GraphStateList::iterator it;
		for(it = open_queue_.begin() ; it != open_queue_.end() ; it++)
		{
			if(graph_state.getId() == it->getId())
			{
				graph_state_in_open = *it;
				break;
			}
		}

		return graph_state_in_open;
	}


	bool WeightedAStarPlanner::isGoalConditionSatisfied(GraphState graph_state)
	{
		GraphState goal_state = graph_state_manager_->getGoalState();

		std::vector<double> graph_state_variables = graph_state.getStateVariables();
		std::vector<double> goal_state_variables = goal_state.getStateVariables();

		if( (std::abs(goal_state_variables[0] - graph_state_variables[0]) < ak_planner_shared_variables::goal_region_x_threshold) &&
			(std::abs(goal_state_variables[1] - graph_state_variables[1]) < ak_planner_shared_variables::goal_region_y_threshold) &&
			(std::abs(goal_state_variables[2] - graph_state_variables[2]) < ak_planner_shared_variables::goal_region_theta_threshold) )
		{
			return true;
		}
		else
		{
			return false;
		}

	}


	double WeightedAStarPlanner::getHeuristicValue(GraphState graph_state)
	{

		double bfs_heur_value;
		double dubins_heur_value;

		bfs_heur_value = bfs_2d_heuristic_->getGoalHeuristic(graph_state);
		dubins_heur_value = dubins_heuristic_->getGoalHeuristic(graph_state);

		if(bfs_heur_value > dubins_heur_value)
		{
			return bfs_heur_value;
		}
		else
		{
			return dubins_heur_value;
		}
	}


	std::vector<GraphState> WeightedAStarPlanner::getSuccessors(GraphState graph_state)
	{
		std::vector<GraphState> successor_states;


		// GraphState successor1;
		// std::vector<double> state_variables1 = graph_state.getStateVariables();
		// state_variables1[0] = state_variables1[0] + 0.2;
		// successor1.setPrimitiveId(1);
		// successor1.setStateType(GraphState::SINGLE_POINT);
		// successor1.setStateVariables(state_variables1[0], state_variables1[1], state_variables1[2]);

		// int x_disc1, y_disc1, theta_disc1;
		// getDiscretePose(state_variables1[0], state_variables1[1], state_variables1[2], x_disc1, y_disc1, theta_disc1);
		// successor1.setStateVariablesDiscrete(x_disc1, y_disc1, theta_disc1);


		// GraphState successor2;
		// std::vector<double> state_variables2 = graph_state.getStateVariables();
		// state_variables2[0] = state_variables2[0] - 0.2;
		// successor2.setPrimitiveId(2);
		// successor2.setStateType(GraphState::SINGLE_POINT);
		// successor2.setStateVariables(state_variables2[0], state_variables2[1], state_variables2[2]);

		// int x_disc2, y_disc2, theta_disc2;
		// getDiscretePose(state_variables2[0], state_variables2[1], state_variables2[2], x_disc2, y_disc2, theta_disc2);
		// successor2.setStateVariablesDiscrete(x_disc2, y_disc2, theta_disc2);



		// std::vector<double> parent_state_variables = graph_state.getStateVariables();

		// int num_of_primitives = motion_primitives_->getNumOfPrim();
		// for(int i = 0 ; i < num_of_primitives ; i++)
		// {
		// 	double x_new;
		// 	double y_new;
		// 	double theta_new;

		// 	motion_primitives_->applyPrimitive(i, parent_state_variables[0], parent_state_variables[1], parent_state_variables[2], 
		// 		x_new, y_new, theta_new);

			
		// 	GraphState successor_state;
		// 	successor_state.setPrimitiveId(i);
		// 	successor_state.setStateType(GraphState::SINGLE_POINT);
		// 	successor_state.setStateVariables(x_new, y_new, theta_new);

		// 	int x_disc, y_disc, theta_disc;
		// 	// std::cout << "x_disc: " << x_disc << "y_disc: " << y_disc << "theta_disc: " << theta_disc << std::endl;
		// 	getDiscretePose(x_new, y_new, theta_new, x_disc, y_disc, theta_disc);
		// 	// std::cout << "x_disc: " << x_disc << "y_disc: " << y_disc << "theta_disc: " << theta_disc << std::endl;			
		// 	successor_state.setStateVariablesDiscrete(x_disc, y_disc, theta_disc);

		// 	successor_states.push_back(successor_state);

		// }


		// Starting to incorporate Parser.

		int x_disc, y_disc, theta_disc;
		std::vector<double> state_variables;
		state_variables = graph_state.getStateVariables();
		graph_state.getStateVariablesDiscrete(x_disc, y_disc, theta_disc);

		std::vector<TaskSpacePrimitive> primitives_list = motion_primitives_parser_->getPrimitiveListFromStartAngleID(theta_disc);

		for(int i=0 ; i<primitives_list.size() ; i++)
		{
			TaskSpacePrimitive primitive = primitives_list[i];

			int x_disc_new, y_disc_new, theta_disc_new;
			double x_cont_new, y_cont_new, theta_cont_new;

			x_disc_new = x_disc + primitive.prim_disc_end_pose[0];
			y_disc_new = y_disc + primitive.prim_disc_end_pose[1];
			theta_disc_new = ((primitive.prim_disc_end_pose[2] + motion_primitives_parser_->num_of_angles_) % motion_primitives_parser_->num_of_angles_) ;

			if(theta_disc_new < 0)
			{
				std::cout << "Error here. theta_disc_new: " << theta_disc_new << std::endl;
				throw 0;
			}

			std::vector<double> cont_end_pose = primitive.prim_intermediate_poses.back();
			x_cont_new = state_variables[0] + cont_end_pose[0];
			y_cont_new = state_variables[1] + cont_end_pose[1];
			theta_cont_new = cont_end_pose[2];

			
			GraphState successor_state;
			successor_state.setPrimitiveId(primitive.prim_id);
			successor_state.setStateType(GraphState::SINGLE_POINT);
			successor_state.setStateVariables(x_cont_new, y_cont_new, theta_cont_new);
			successor_state.setStateVariablesDiscrete(x_disc_new, y_disc_new, theta_disc_new);

			//For interpolation purposes once path is obtained.
			// successor_state.prim_intermediate_poses_ = primitive.prim_intermediate_poses;


			successor_states.push_back(successor_state);
			
			
		}



		// successor_states.push_back(successor1);
		// successor_states.push_back(successor2);




		return successor_states;

	}


	bool WeightedAStarPlanner::isInEnvironmentBounds(GraphState& graph_state)
	{
		
		std::vector<double> state_variables = graph_state.getStateVariables();

		if( (state_variables[0] <= occupancy_grid_2d_->origin_x_) ||  
			(state_variables[0] >= occupancy_grid_2d_->origin_x_ + occupancy_grid_2d_->size_x_) ||
			(state_variables[1] <= occupancy_grid_2d_->origin_y_) ||
			(state_variables[1] >= occupancy_grid_2d_->origin_y_ + occupancy_grid_2d_->size_y_) )
		{
			return false;
		}

		return true;
	}



	void WeightedAStarPlanner::expandGraphState(GraphState current_graph_state)
	{

		if( isStateClosed(current_graph_state) )
		{
			ROS_ERROR_STREAM("Expanding an already closed state in expandGraphState - expandGraphState");
			throw std::runtime_error("Expanding an already closed state in expandGraphState - expandGraphState");
		}

		closed_list_.push_back(current_graph_state);

		if( isGoalConditionSatisfied(current_graph_state) )
		{
			is_goal_state_expanded_ = true;
			std::cout << "GOAL STATE FOUND!" << std::endl;
			expanded_goal_state_ = current_graph_state;
		}


		std::vector<GraphState> successors = getSuccessors(current_graph_state); // TODO: implement getSuccessors in graph state manager. Set all parameters except for Id, h/g/f_vals, pred_id in that function but dont add it to the hash map yet.

		// std::cout << "successors.size(): " << successors.size() << std::endl;


		for(int i = 0 ; i < (int)successors.size() ; i++)
		{
			GraphState successor_state = successors[i];

			std::vector<double> state_variables = successor_state.getStateVariables();
			bool is_collision = moveit_interface_->isStateInCollision(state_variables[0], state_variables[1], state_variables[2]);

			if( !is_collision && isInEnvironmentBounds(successor_state) )
			{

				int state_id = graph_state_manager_->getStateId(successor_state);
				if(state_id == GraphStateManager::STATE_NONEXISTENT)
				{
					// std::cout << "State not in existence. ID set for state: " << graph_state_manager_->getCurrentId() << std::endl;
					successor_state.setId( graph_state_manager_->getCurrentId() );
				}
				else
				{
					successor_state.setId(state_id);
				}

				successor_state.setHVal( getHeuristicValue(successor_state) );

				if( !isStateClosed(successor_state) )
				{
					GraphState similar_state_in_open;

					if(isPresentInOpen(successor_state))
					{
						// std::cout << "State present in open." << std::endl;
						similar_state_in_open = getSimilarStateFromOpen(successor_state);
					}
					else
					{
						// std::cout << "State not present in open." << std::endl;
						similar_state_in_open = successor_state;
					}

					if(similar_state_in_open.getGVal() > current_graph_state.getGVal() + cost_prim_)
					{
						successor_state.setGVal( (current_graph_state.getGVal() + cost_prim_) );
						successor_state.setFVal( (successor_state.getGVal() + successor_state.getHVal()) );
						successor_state.setPredId( (current_graph_state.getId()) );

						if(state_id == GraphStateManager::STATE_NONEXISTENT)
						{
							graph_state_manager_->addGraphState(successor_state);
						}

						if(verbosity_)
						{
							std::cout << "=>" << std::endl;
							successor_state.printStateAttributes();
							std::cout << std::endl;
						}

						insertStateInOpenQueue(successor_state);
					}
				}

				else
				{
					if(verbosity_)
					{
						std::cout << "=>" << std::endl;
						std::cout << "State in already closed. State ID: " << successor_state.getId() << std::endl << std::endl;
					}

				}
			}

			else
			{
				if(verbosity_)
				{
					// std::cout << "State in collision. Rejecting State." << std::endl;
					std::cout << "=>State in collision. Rejecting State." << std::endl;
					successor_state.printStateAttributes();
					std::cout << std::endl;
				}

			}

		} // end successor_states for loop

		if(verbosity_)
		{
			std::cout << std::endl << std::endl << std::endl;
		}

	}


	void WeightedAStarPlanner::getSolutionPath()
	{
		std::cout << std::endl << "Getting solution path. " << std::endl;

		solution_path_.insert(solution_path_.begin(), expanded_goal_state_);

		expanded_goal_state_.printStateAttributes();

		int pred_id = expanded_goal_state_.getPredId();
		while(pred_id != GraphStateManager::START_STATE_ID)
		{
			GraphState graph_state = graph_state_manager_->getGraphState(pred_id); // TODO: the state got from bimap may not have the updated g,h,f vals.
			solution_path_.insert(solution_path_.begin(), graph_state);
			// std::cout << "pred_id: " << pred_id << std::endl;
			pred_id = graph_state.getPredId();
		}

		solution_path_.insert(solution_path_.begin(), graph_state_manager_->getGraphState(GraphStateManager::START_STATE_ID));

		// std::cout << "OPEN queue size: " << open_queue_.size() << std::endl;
		// std::cout << "CLOSED list size: " << closed_list_.size() << std::endl;
		// std::cout << "HASH-BIMAP size: " << graph_state_manager_->getBimapSize() << std::endl;
		

	}

	void WeightedAStarPlanner::printSolutionPathStates()
	{
		std::cout << "Printing Path State Attributes: " << std::endl << std::endl;

		for(int i=0 ; i<solution_path_.size() ; i++)
		{
			GraphState graph_state = solution_path_[i];
			std::cout << "=>" << std::endl;
			graph_state.printStateAttributes();
			std::cout << std::endl;
		}
	}


	void WeightedAStarPlanner::getInterpolatedSolutionPath(std::vector<std::vector<double> >& interpolated_path)
	{

		std::cout << "Interpolating Solution Path. Solution path size: " << solution_path_.size() << std::endl;


		for(int i=0 ; i<solution_path_.size()-1 ; i++)
		{
			std::cout << "Solution path node number: " << i << std::endl;

			GraphState pred_state = solution_path_[i];
			GraphState succ_state = solution_path_[i+1];

			int dummy_x, dummy_y, predecessor_angle_id;
			pred_state.getStateVariablesDiscrete(dummy_x, dummy_y, predecessor_angle_id);
			int successor_prim_id = succ_state.getPrimitiveId();
			
			std::vector<std::vector<double> > interm_poses = motion_primitives_parser_->getPrimitiveIntermediatePoses(predecessor_angle_id, successor_prim_id);
			
			//Debugging
			// std::cout << "Here 1" << std::endl;
			// for(int k=0 ; k<interm_poses.size() ; k++)
			// {
			// 	std::vector<double> pose = interm_poses[k];
			// 	std::cout << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
			// }
			// char c;
			// std::cin >> c;
			// std::cout << "Here 2" << std::endl;
			//

			std::vector<double> predecessor_state_variables = pred_state.getStateVariables();
	
			for(int j=0 ; j<interm_poses.size()-1 ; j++)
			{
				std::vector<double> interpolated_pose;
				interpolated_pose.resize(3);

				std::vector<double> interm_pose = interm_poses[j];

				interpolated_pose[0] = predecessor_state_variables[0] + interm_pose[0];
				interpolated_pose[1] = predecessor_state_variables[1] + interm_pose[1];
				interpolated_pose[2] = interm_pose[2];

				interpolated_path.push_back(interpolated_pose);
			}

		}

		GraphState path_last_state = solution_path_.back();
		std::vector<double> last_state_variables = path_last_state.getStateVariables();

		interpolated_path.push_back(last_state_variables);

		std::cout << "Path interpolation done. Interpolated path size: " << interpolated_path.size() << std::endl;

	}


	bool WeightedAStarPlanner::plan(const double start_state_x, const double start_state_y, const double start_state_theta, 
			const double goal_state_x, const double goal_state_y, const double goal_state_theta, bool verbosity)
	{

		verbosity_ = verbosity;

		initPlanner(start_state_x, start_state_y, start_state_theta, goal_state_x, goal_state_y, goal_state_theta);

		if(verbosity_)
		{
			std::cout << "open_queue_.size(): " << open_queue_.size() << std::endl;
		}


		while( !(is_goal_state_expanded_) && !(open_queue_.size()==0) )
		{
			GraphState min_f_graph_state = popMinFValStateFromQueue();

			if(verbosity_)
			{
				std::cout << "OPEN QUEUE SIZE: " << open_queue_.size() << "\t\t";
				std::cout << "EXPANDING STATE ID: " << min_f_graph_state.getId() << std::endl << std::endl;
			}

			expandGraphState(min_f_graph_state);
		}

		if(is_goal_state_expanded_)
		{
			//Shukar Alhumdulillah
			std::cout << "Getting solution Path" << std::endl;
			getSolutionPath();
			std::cout << "PATH FOUND!" << std::endl;
			return true;
		}
		else
		{
			std::cout << "Path not found." << std::endl;
			return false;
		}
	}




} // end namespace ak_planner