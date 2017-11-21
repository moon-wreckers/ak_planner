/*
 * Author: 	Abdul Moeed Zafar
 * Date:	20 Nov, 2017
 *
 * File:	motion_primitives_parser.h
 */

#ifndef _MOTION_PRIMITIVES_PARSER_
#define _MOTION_PRIMITIVES_PARSER_


#include <ros/package.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <utility>
#include <cmath>


namespace ak_planner
{

	typedef std::pair<int, double> AngleIDValuePair;

	struct TaskSpacePrimitive
	{
		int prim_id;
		int prim_start_angle_id;
		std::vector<int> prim_disc_end_pose;
		
		double prim_action_cost_mult;
		double prim_turning_radius;

		int prim_num_of_intermediate_poses;
		std::vector<std::vector<double> > prim_intermediate_poses;

		

		TaskSpacePrimitive()
		{
			prim_id = -1;
			prim_start_angle_id = -1;
			prim_action_cost_mult = -1;
			prim_turning_radius = -1;
			prim_num_of_intermediate_poses = -1;

			prim_disc_end_pose.resize(3);
		}

		void printPrimitive()
		{
			std::cout << "Primitive ID: " << prim_id << std::endl;
			std::cout << "Primitive Start Angle: " << prim_start_angle_id << std::endl;
			std::cout << "Primitive Discrete End Pose: " << prim_disc_end_pose[0] << " " << prim_disc_end_pose[1] << " " << prim_disc_end_pose[2] << std::endl;
			std::cout << "Primitive Action Cost Multi: " << prim_action_cost_mult << std::endl;
			std::cout << "Primitve Turning Radius: " << prim_turning_radius << std::endl;
			std::cout << "Primitive Num of Intermediate Poses: " << prim_num_of_intermediate_poses << std::endl;
			std::cout << "Poses: " << std::endl;

			for(int i=0 ; i<prim_intermediate_poses.size() ; i++)
			{
				std::vector<double> pose = prim_intermediate_poses[i];
				std::cout << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
			}
		}
	};



	class MotionPrimitivesParser
	{

	public:

		MotionPrimitivesParser();
		~MotionPrimitivesParser() {}

		void readPrimitives();
		void printPrimitives();

		std::vector<TaskSpacePrimitive> getPrimitiveListFromStartAngleID(int start_angle);

		int discretizeAngle(double angle_double);

		std::vector<std::vector<double> > getPrimitiveIntermediatePoses(int predecessor_angle_id, int successor_prim_id);

	// private:

		double prim_resolution_;
		double prim_min_radius_;
		int num_of_angles_;
		std::vector<AngleIDValuePair> angle_id_value_pair_list_;
		int num_of_prim_;

		std::vector<TaskSpacePrimitive> primitives_list_;

		std::string file_path_;
		std::ifstream file_stream_;

		bool primitives_loaded_;

	};

	typedef boost::shared_ptr<MotionPrimitivesParser> MotionPrimitivesParserPtr;

} // end namespace ak_planenr


#endif