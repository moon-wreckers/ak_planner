/*
 * Author: 	Abdul Moeed Zafar
 * Date:	8 Nov, 2017
 *
 * File:	graph_state.h
 */


#ifndef _GRAPH_STATE_H_
#define _GRAPH_STATE_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace ak_planner
{

	class GraphState
	{		

	public:

		enum GraphStateType
		{
			DUMMY = 0, SINGLE_POINT, UNINITIALIZED
		};

		GraphState();
		~GraphState();

		void setId(const int& id);
		
		inline void setPrimitiveId(const int& prim_id);
		inline void setPredId(const int& pred_id);
		inline void setStateType(const GraphStateType& state_type);
		inline void setStateVariables(const double x, const double y, const double theta);
		inline bool isValid() const;
		inline bool allParamsInit() const;
		
		bool operator < (const GraphState& other) const;

		inline int getId() const;
		inline int getPrimitiveId() const;
		inline int getPredId() const;
		inline GraphStateType getStateType() const;
		inline std::vector<double> getStateVariables() const;

		void printStateAttributes() const;

		static GraphState getDummyState();

		inline void setGVal(double gval);
		inline void setHVal(double hval);
		inline void setFVal(double fval);
		inline double getGVal();
		inline double getHVal();
		inline double getFVal() const;

		inline void setStateVariablesDiscrete(const int x_coord, const int y_coord, const int theta_coord);
		inline void getStateVariablesDiscrete(int& x_coord, int& y_coord, int& theta_coord);
		

		//For interpolation purposes once path is obtained.
		// std::vector<std::vector<double> > prim_intermediate_poses_;

	private:

		int id_;
		int prim_id_;
		int pred_id_;
		GraphStateType state_type_;
		std::vector<double> state_variables_; // state_variables_[0]: x, state_variables_[1]: y, state_variables_[2]: theta

		int x_coord_;
		int y_coord_;
		int theta_coord_;

		double fval_;
		double gval_;
		double hval_;

		bool id_init_;
		bool prim_id_init_;
		bool pred_id_init_;
		bool state_type_init_;
		bool state_variables_init_;
		bool state_variables_discrete_init_;

	};


	
	typedef boost::shared_ptr<GraphState> GraphStatePtr;
	typedef std::vector<GraphState> GraphStateList;

	

	inline void GraphState::setPrimitiveId(const int& prim_id)
	{
		prim_id_ = prim_id;
		prim_id_init_ = true;
	}

	inline void GraphState::setPredId(const int& pred_id)
	{
		pred_id_ = pred_id;
		pred_id_init_ = true;
	}

	inline void GraphState::setStateType(const GraphStateType& state_type)
	{
		assert(state_type != UNINITIALIZED);
		state_type_ = state_type;
		state_type_init_ = true;
	}

	inline void GraphState::setStateVariables(const double x, const double y, const double theta)
	{
		//assert(state_variables.size() == 3);
		state_variables_[0] = x;
		state_variables_[1] = y;
		state_variables_[2] = theta;
		state_variables_init_ = true;
	}

	inline bool GraphState::isValid() const
	{
		return ( !(state_type_ == DUMMY) && !(state_type_ == UNINITIALIZED));
	}

	inline bool GraphState::allParamsInit() const
	{
		return ( id_init_ && prim_id_init_ && state_variables_init_ && pred_id_init_ && state_type_init_ && state_variables_discrete_init_);
	}

	inline int GraphState::getId() const
	{
		assert(id_init_);
		return id_;
	}

	inline int GraphState::getPrimitiveId() const
	{
		assert(prim_id_init_);
		return prim_id_;
	}

	inline int GraphState::getPredId() const
	{
		assert(pred_id_init_);
		return pred_id_;
	}

	inline GraphState::GraphStateType GraphState::getStateType() const
	{
		assert(state_type_init_);
		return state_type_;
	}

	inline std::vector<double> GraphState::getStateVariables() const
	{
		assert(state_variables_init_);
		return state_variables_;
	}



	inline void GraphState::setGVal(double gval)
	{
		gval_ = gval;
	}

	inline void GraphState::setHVal(double hval)
	{
		hval_ = hval;
	}

	inline void GraphState::setFVal(double fval)
	{
		fval_ = fval;
	}

	inline double GraphState::getGVal()
	{
		return gval_;
	}

	inline double GraphState::getHVal()
	{
		return hval_;
	}

	inline double GraphState::getFVal() const
	{
		return fval_;
	}



	inline void GraphState::setStateVariablesDiscrete(const int x_coord, const int y_coord, const int theta_coord)
	{
		x_coord_ = x_coord;
		y_coord_ = y_coord;
		theta_coord_ = theta_coord;
		state_variables_discrete_init_ = true;
	}

	inline void GraphState::getStateVariablesDiscrete(int& x_coord, int& y_coord, int& theta_coord)
	{
		assert(state_variables_discrete_init_);
		x_coord = x_coord_;
		y_coord = y_coord_;
		theta_coord = theta_coord_;
	}


} // end namespace ak_planner

#endif