/*
 * Author: 	Abdul Moeed Zafar
 * Date:	28 Oct, 2017
 *
 * File:	shared_variables.h
 */


#ifndef _SHARED_VARIABLES_H_
#define _SHARED_VARIABLES_H_

#include <ros/ros.h>

namespace rover_simulation
{

	/*
	 *	ENVIRONMENT_OBSTACLES
	 *	Description: To add obstacle to environment, call addObstacle function in the constructor with obstacle specification as arguments.
	 */
	struct EnvironmentObstacles
	{
		std::vector<std::vector<double> > obstacles;

		EnvironmentObstacles()
		{

			// addObstacle(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5);
			// addObstacle(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5);
		}


		void addObstacle(double x, double y, double z, double orientation_x, double orientation_y, double orientation_z,
			double size_x, double size_y, double size_z)
		{
			float obstacle_specs[9];
			obstacle_specs[0] = x;
			obstacle_specs[1] = y;
			obstacle_specs[2] = z;
			obstacle_specs[3] = orientation_x;
			obstacle_specs[4] = orientation_y;
			obstacle_specs[5] = orientation_z;
			obstacle_specs[6] = size_x;
			obstacle_specs[7] = size_y;
			obstacle_specs[8] = size_z;

			std::vector<double> obstacle(obstacle_specs, obstacle_specs + (sizeof(obstacle_specs)/sizeof(obstacle_specs[0])));
			obstacles.push_back(obstacle);
		}

	} const ENVIRONMENT_OBSTACLES; 



	/*
	 *	Constant offset in z-axis to position rover on the ground.
	 */
	const float ODOM_BASE_TF_Z = 0.08;





} // end namespace rover_simulation

#endif