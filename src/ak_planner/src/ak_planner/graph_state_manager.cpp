/*
 * Author: 	Abdul Moeed Zafar
 * Date:	9 Nov, 2017
 *
 * File:	graph_state_manager.cpp
 */

#include <ak_planner/graph_state_manager.h>

namespace ak_planner
{
	const int GraphStateManager::STATE_NONEXISTENT = -1;
	const int GraphStateManager::START_STATE_ID = 1;
	const int GraphStateManager::GOAL_STATE_ID = 0;
	

	GraphStateManager::GraphStateManager() : running_id_(START_STATE_ID+1), goal_state_added_(false)
	{

	}

	GraphStateManager::~GraphStateManager()
	{

	}

	GraphState GraphStateManager::getGraphState(const int& state_id) const
	{
		IDGraphStateBimap::left_map::const_iterator iter = id_graph_state_bimap_.left.find(state_id);
		if(iter != id_graph_state_bimap_.left.end())
		{
			return iter->second;
		}
		else
		{
			return GraphState::getDummyState();
		}
	}

	GraphState GraphStateManager::getGoalState() const
	{
		assert (goal_state_added_ == true);
		return getGraphState(GOAL_STATE_ID);
	}

	int GraphStateManager::getStateId(const GraphState& graph_state) const
	{
		IDGraphStateBimap::right_map::const_iterator iter = id_graph_state_bimap_.right.find(graph_state);
		if(iter != id_graph_state_bimap_.right.end())
		{
			return iter->second;
		}
		else
		{
			return STATE_NONEXISTENT;
		}
	}

	int GraphStateManager::addGraphState(GraphState& graph_state)
	{
		int current_id = running_id_++;
		graph_state.setId(current_id);
		
		if(!graph_state.allParamsInit())
		{
			ROS_ERROR_STREAM("One or more graph state parameters have not been set. Cannot add graph state");
			throw std::runtime_error("One or more graph state parameters have not been set. Cannot add graph state");	
		}

		id_graph_state_bimap_.insert(IDGraphStateBimap::value_type(current_id, graph_state));
		return current_id;
	}

	bool GraphStateManager::deleteGraphState(const GraphState& graph_state)
	{
		if(getStateId(graph_state) != STATE_NONEXISTENT)
		{
			id_graph_state_bimap_.right.erase(graph_state);
			return true;
		}
		else
		{
			return false;
		}
	}

	void GraphStateManager::addStartState(GraphState& graph_state)
	{
		if(getGraphState(START_STATE_ID).isValid())
		{
			ROS_ERROR_STREAM("Start state has already been set.");
			throw std::runtime_error("Start state has already been set.");
		}

		graph_state.setId(START_STATE_ID);
		id_graph_state_bimap_.insert(IDGraphStateBimap::value_type(START_STATE_ID, graph_state));
	}

	void GraphStateManager::addGoalState(GraphState& graph_state)
	{
		if(getGraphState(GOAL_STATE_ID).isValid())
		{
			std::cout << "Goal tolerance has already been reached by a previous state. Will not overwrite." << std::endl;
			return;
		}

		graph_state.setId(GOAL_STATE_ID);
		id_graph_state_bimap_.insert(IDGraphStateBimap::value_type(GOAL_STATE_ID, graph_state));
		goal_state_added_ = true;
	}

	void GraphStateManager::reset()
	{
		id_graph_state_bimap_.clear();
		running_id_ = START_STATE_ID + 1;
		goal_state_added_ = false;
	}

	void GraphStateManager::printIDGraphStateMap() const
	{
		std::cout << "There are " << id_graph_state_bimap_.size() << " entries in the graph state map." << std::endl;
		for(IDGraphStateBimap::const_iterator iter = id_graph_state_bimap_.begin(); iter != id_graph_state_bimap_.end(); iter++)
		{
			GraphState graph_state = iter->right;
			std::cout << "---------------------------------------------------------" << std::endl;
			//std::cout << "Bimap ID: " << iter->left << std::endl;
			graph_state.printStateAttributes();
		}
		std::cout << "---------------------------------------------------------" << std::endl;	
	}


}