/*
 * Author: 	Abdul Moeed Zafar
 * Date:	20 Nov, 2017
 *
 * File:	motion_primitives_parser.cpp
 */

#include <ros/ros.h>
#include <ak_planner/motion_primitives_parser.h>


namespace ak_planner
{

	// For changing back to old primitives, do the following:
	// 1) Change the file_path_ for the file name
	// 2) Change the throw error thing corresponding to old file
	// 3) Change num_of_prim_per_angle_
	// 4) Uncomment the primitive.prim_turning_radius portion code

	MotionPrimitivesParser::MotionPrimitivesParser()
	{

		file_path_ = ros::package::getPath("ak_planner");
		file_path_ += "/primitives/autokrawler_v01.txt";
		// file_path_ += "/primitives/non_uniform_res0025_rad1_err005.txt";
	
		file_stream_.open(file_path_.c_str());

		if(!file_stream_.is_open())
		{
			// ROS_ERROR_STREAM("ak_planner/primitives/non_uniform_res0025_rad1_err005.txt could not be opened");
			// throw std::runtime_error("ak_planner/primitives/non_uniform_res0025_rad1_err005.txt could not be opened");
			
			ROS_ERROR_STREAM("ak_planner/primitives/autokrawler_v01.txt could not be opened");
			throw std::runtime_error("ak_planner/primitives/autokrawler_v01.txt could not be opened");
		}
		else
		{
			std::string line;
			std::vector<std::string> tokens;

			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			prim_resolution_ = atof(tokens[1].c_str());

			tokens.clear();
			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			prim_min_radius_ = atof(tokens[1].c_str());

			tokens.clear();
			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			num_of_angles_ = atof(tokens[1].c_str());

			bool is_primitive_info_end = false;
			while(!is_primitive_info_end)
			{

				std::vector<std::string> temp_sub_tokens;

				tokens.clear();
				std::getline(file_stream_, line);
				if(line.at(0) == 't')
				{
					is_primitive_info_end = true;
					continue;
				}

				boost::algorithm::split_regex(tokens, line, boost::regex(":"));
				boost::algorithm::split_regex(temp_sub_tokens, tokens[1], boost::regex(" "));

				int angle_id = atof(temp_sub_tokens[0].c_str());
				double angle_value = atof(temp_sub_tokens[1].c_str());

				AngleIDValuePair angle_id_value_pair(angle_id, angle_value);

				angle_id_value_pair_list_.push_back(angle_id_value_pair);
			}

			primitives_loaded_ = false;

			std::cout << "resolution: " << prim_resolution_ << std::endl;
			std::cout << "min_radius: " << prim_min_radius_ << std::endl;
			std::cout << "num_of_angles_: " << num_of_angles_ << std::endl;
			for(int i=0 ; i < angle_id_value_pair_list_.size() ; i++)
			{
				std::cout << "Angle ID: " << angle_id_value_pair_list_[i].first << "\tValue: " << angle_id_value_pair_list_[i].second << std::endl;
			}
		}

		// num_of_prim_per_angle_ = 8;		// For old primitives
		num_of_prim_per_angle_ = 11;		// For new primitives (autokrawler_v01.txt)

	}


	void MotionPrimitivesParser::readPrimitives()
	{
		bool reached_end_of_file = false;

		int iteration_count = 1;

		while(!reached_end_of_file)
		{


			// std::cout << "Here 1" << std::endl;

			std::string line;
			std::vector<std::string> tokens;
			std::vector<std::string> sub_tokens;
			TaskSpacePrimitive primitive;
			
			std::getline(file_stream_, line);
			if(line.at(0) == 'e')
			{
				reached_end_of_file = true;
				continue;
			}
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			primitive.prim_id = atof(tokens[1].c_str());

			tokens.clear();
			sub_tokens.clear();
			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			primitive.prim_start_angle_id = atof(tokens[1].c_str());

			//Debuging
			// if(primitive.prim_start_angle_id == 1)
			// {
			// 	break;
			// }

			tokens.clear();
			sub_tokens.clear();
			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			boost::algorithm::split_regex(sub_tokens, tokens[1], boost::regex(" "));
			primitive.prim_disc_end_pose[0] = atof(sub_tokens[0].c_str());
			primitive.prim_disc_end_pose[1] = atof(sub_tokens[1].c_str());
			primitive.prim_disc_end_pose[2] = atof(sub_tokens[2].c_str());

			tokens.clear();
			sub_tokens.clear();
			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			primitive.prim_action_cost_mult = atof(tokens[1].c_str());

			
			// Uncomment below part for old primitive file
			primitive.prim_turning_radius = 0.0000;
			// tokens.clear();
			// std::getline(file_stream_, line);
			// boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			// primitive.prim_turning_radius = atof(tokens[1].c_str());

			tokens.clear();
			std::getline(file_stream_, line);
			boost::algorithm::split_regex(tokens, line, boost::regex(": "));
			primitive.prim_num_of_intermediate_poses = atof(tokens[1].c_str());

			for(int i=0 ; i<primitive.prim_num_of_intermediate_poses ; i++)
			{
				tokens.clear();
				sub_tokens.clear();
				std::getline(file_stream_, line);
				boost::algorithm::split_regex(tokens, line, boost::regex(" "));

				std::vector<double> intermediate_pose;
				intermediate_pose.resize(3);
				intermediate_pose[0] = atof(tokens[0].c_str());
				intermediate_pose[1] = atof(tokens[1].c_str());
				intermediate_pose[2] = atof(tokens[2].c_str());

				primitive.prim_intermediate_poses.push_back(intermediate_pose);

			}

			// primitive.printPrimitive();

			if(primitive.prim_disc_end_pose[0] == 0 && 
				primitive.prim_disc_end_pose[1] == 0 ) 
			{	
				// Skip adding primitives that enables changing angles in place
			}
			else
			{
				primitives_list_.push_back(primitive);
			}

			primitives_loaded_ = true;
			num_of_prim_ = primitives_list_.size();

			// std::cout << "Here 2" << std::endl;
			// std::cout << "Iteration: " << iteration_count << std::endl;
			// iteration_count++;
			
		}

	}

	void MotionPrimitivesParser::printPrimitives()
	{
		for(int i=0 ; i<primitives_list_.size() ; i++)
		{
			primitives_list_[i].printPrimitive();
			std::cout << std::endl;
		}

		std::cout << "Primitive List size: " << primitives_list_.size() << std::endl;

	}


	std::vector<TaskSpacePrimitive> MotionPrimitivesParser::getPrimitiveListFromStartAngleID(int start_angle)
	{
		if(!primitives_loaded_)
		{
			ROS_ERROR_STREAM("Primitives not loaded from the file. ");
			throw std::runtime_error("Primitives not loaded from the file. ");
		}

		if(start_angle >= num_of_angles_)
		{
			std::cout << "Total num of angles: " << num_of_angles_ << std::endl;
			std::cout << "Primitives asked for start angle ID: " << start_angle << std::endl;
			ROS_ERROR_STREAM("Primitives not found in the primitives list for the given start_angle. ");
			throw std::runtime_error("Primitives not found in the primitives list for the given start_angle. ");
		}

		int offset_1 = start_angle * num_of_prim_per_angle_;
		int offset_2 = offset_1 + num_of_prim_per_angle_;

		std::vector<TaskSpacePrimitive> primitives_list_required(primitives_list_.begin() + offset_1, primitives_list_.begin() + offset_2);
		return primitives_list_required;
	}

	int MotionPrimitivesParser::discretizeAngle(double angle_double)
	{	
		if (angle_double < 0.0) 
		{
	        angle_double += 2 * M_PI;
	    }

		std::vector<AngleIDValuePair>::iterator it_current;

		for(it_current = angle_id_value_pair_list_.begin() ; it_current < (angle_id_value_pair_list_.end()-1) ; it_current++)
		{
			if( (angle_double >= it_current->second) && (angle_double < (it_current+1)->second))
			{
				return it_current->first;
			}
		}

		if( (angle_double >= it_current->second) && (angle_double < 6.28318) )
		{
			return it_current->first;
		}

		//Shouldnt have reached to this point in the function. Something wrong somewhere.
		std::cout << "Angle to be discretized: " << angle_double << std::endl;
		ROS_ERROR_STREAM("Not able to discretize the given angle value based on the Angle ID-Value pair obtained from parsed file. ");
		throw std::runtime_error("Not able to discretize the given angle value based on the Angle ID-Value pair obtained from parsed file. ");

		return -1;
	}


	std::vector<std::vector<double> > MotionPrimitivesParser::getPrimitiveIntermediatePoses(int predecessor_angle_id, int successor_prim_id)
	{
		std::vector<TaskSpacePrimitive> primitives_list = getPrimitiveListFromStartAngleID(predecessor_angle_id);

		bool primitive_found = false;

		for(int i=0 ; i<primitives_list.size() ; i++)
		{
			TaskSpacePrimitive primitive = primitives_list[i];
			if(primitive.prim_id == successor_prim_id)
			{
				//Debugging
				// primitive.printPrimitive();
				// char c;
				// std::cin >> c;
				//

				
				primitive_found = true;
				return primitive.prim_intermediate_poses;
			}
		}

		if(!primitive_found)
		{
			ROS_ERROR_STREAM("Primitive should have been found in the primitive list when interpolating. ");
			throw std::runtime_error("Primitive should have been found in the primitive list when interpolating. ");
		}

	}



} // end namespace ak_planner