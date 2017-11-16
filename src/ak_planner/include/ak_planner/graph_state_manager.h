/*
 * Author: 	Abdul Moeed Zafar
 * Date:	9 Nov, 2017
 *
 * File:	graph_state_manager.h
 */

#ifndef _GRAPH_STATE_MANAGER_H_
#define _GRAPH_STATE_MANAGER_H_

#include <ros/ros.h>
#include <ak_planner/graph_state.h>
#include <boost/bimap.hpp>

namespace ak_planner
{

	typedef boost::bimap<int, GraphState> IDGraphStateBimap;

	class GraphStateManager
	{

	public:
		
		GraphStateManager();
		~GraphStateManager();

		GraphState getGraphState(const int& state_id) const;
		GraphState getGoalState() const;
		int getStateId(const GraphState& graph_state) const;
		int addGraphState(GraphState& graph_state);
		bool deleteGraphState(const GraphState& graph_state);
		void addStartState(GraphState& graph_state);
		void addGoalState(GraphState& graph_state);
		inline bool isGoalStateAdded() const;
		void reset();
		void printIDGraphStateMap() const;

		inline int getCurrentId() const;
		inline int getBimapSize() const;

		static const int STATE_NONEXISTENT;
		static const int START_STATE_ID;
		static const int GOAL_STATE_ID;

	private:

		int running_id_;
		IDGraphStateBimap id_graph_state_bimap_;
		bool goal_state_added_;

	};


	typedef boost::shared_ptr<GraphStateManager> GraphStateManagerPtr;


	inline bool GraphStateManager::isGoalStateAdded() const
	{
		return goal_state_added_;
	}

	inline int GraphStateManager::getCurrentId() const
	{
		return running_id_;
	}

	inline int GraphStateManager::getBimapSize() const
	{
		return id_graph_state_bimap_.size();
	}


} // end namespace ak_planner

#endif