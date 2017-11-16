/*
 * Author: 	Abdul Moeed Zafar
 * Date:	8 Nov, 2017
 *
 * File:	graph_state.cpp
 */

#include <ak_planner/graph_state.h>

namespace ak_planner
{

	GraphState::GraphState() : id_(-1), prim_id_(-1), pred_id_(-1), state_type_(UNINITIALIZED), id_init_(false), prim_id_init_(false),
		pred_id_init_(false), state_type_init_(false), state_variables_init_(false), state_variables_discrete_init_(false)
	{
		//std::cout << "Graph State Constructor called" << std::endl;
		state_variables_.resize(3);

		hval_ = -1;
		gval_ = std::numeric_limits<double>::max()-1000.0;
		fval_ = gval_ + hval_;

	}

	GraphState::~GraphState()
	{

	}

	void GraphState::setId(const int& id)
	{
		if (id < 0)
		{
			ROS_ERROR_STREAM("Graph state ID cannot be less than zero");
			throw std::runtime_error("Graph state ID cannot be less than zero");
		}

		id_ = id;
		id_init_ = true;
	}
	

	bool GraphState::operator < (const GraphState& other) const
	{
		if (this->state_type_ == DUMMY || other.state_type_ == DUMMY)
		{
			ROS_ERROR_STREAM("Operator '<'' is not valid for dummy states");
			throw std::runtime_error("Operator '<' is not valid for dummy states");
		}

		if(this->x_coord_ < other.x_coord_)
		{
			return true;
		}
		else if(this->x_coord_ == other.x_coord_)
		{
			
			if(this->y_coord_ < other.y_coord_)
			{
				return true;
			}			
			else if(this->y_coord_ == other.y_coord_)
			{
				
				if(this->theta_coord_ < other.theta_coord_)
				{
					return true;
				}	
				else
				{
					return false;
				}

			}	
			else
			{
				return false;
			}

		}
		else
		{
			return false;
		}

	}


	void GraphState::printStateAttributes() const
	{
		std::cout << "ID: " << id_ << "\t\t" << "Prim ID: " << prim_id_ << "\t\t" << "Pred ID: " << pred_id_ << std::endl;

		std::cout << "State Variables - x: " << state_variables_[0] << "\ty: " << state_variables_[1] << "\t\ttheta: " << state_variables_[2] << ", " << (state_variables_[2]*180/3.142) << std::endl; 
		std::cout << "State Var discr - x: " << x_coord_ << "\ty: " << y_coord_ << "\t\ttheta: " << theta_coord_ << std::endl; 
		std::cout << "F Val: " << fval_ << "\t\tG Val: " << gval_ << "\t\tH Val: " << hval_ << std::endl;
	
	}


	GraphState GraphState::getDummyState()
	{
		GraphState dummy_state;
		dummy_state.setStateType(GraphState::DUMMY);
		return dummy_state;
	}


} // end namespace ak_planner