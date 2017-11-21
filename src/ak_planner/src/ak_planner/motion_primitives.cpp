/*
 * Author: 	Abdul Moeed Zafar
 * Date:	14 Nov, 2017
 *
 * File:	motion_primitives.cpp
 */

#include <ak_planner/motion_primitives.h>
#include <math.h>


namespace ak_planner
{

	MotionPrimitives::MotionPrimitives()
	{
		initPrimitives();
	}

	MotionPrimitives::~MotionPrimitives()
	{

	}

	void MotionPrimitives::initPrimitives()
	{
		Primitive primitive_0;
		primitive_0.prim_id = 0;
		primitive_0.delta_x = 0.3; //0.5
		primitive_0.delta_y	= 0.0;
		primitive_0.delta_theta =  0.0 * (M_PI/180.0);

		Primitive primitive_1;
		primitive_1.prim_id = 1;
		primitive_1.delta_x = -0.3; //-0.5
		primitive_1.delta_y	=  0.0;
		primitive_1.delta_theta =  0.0 * (M_PI/180.0); 

		Primitive primitive_2;
		primitive_2.prim_id = 2;
		primitive_2.delta_x = 1.0;
		primitive_2.delta_y	= 0.0;
		primitive_2.delta_theta =  0.0 * (M_PI/180.0);

		Primitive primitive_3;
		primitive_3.prim_id = 3;
		primitive_3.delta_x =  0.7;
		primitive_3.delta_y	= -0.15; //-0.3
		primitive_3.delta_theta = -5.0 * (M_PI/180.0); //-10.0

		Primitive primitive_4;
		primitive_4.prim_id = 4;
		primitive_4.delta_x = 0.7;
		primitive_4.delta_y	= 0.15; //0.3
		primitive_4.delta_theta = 5.0 * (M_PI/180.0); //10.0

		Primitive primitive_5;
		primitive_5.prim_id = 5;
		primitive_5.delta_x =  0.7;
		primitive_5.delta_y	= -0.2; //-0.3
		primitive_5.delta_theta = -15.0 * (M_PI/180.0); //-30.0

		Primitive primitive_6;
		primitive_6.prim_id = 6;
		primitive_6.delta_x = 0.7;
		primitive_6.delta_y	= 0.2; //0.3
		primitive_6.delta_theta = 15.0 * (M_PI/180.0); //30.0

		Primitive primitive_7;
		primitive_7.prim_id = 7;
		primitive_7.delta_x =  0.7;
		primitive_7.delta_y	= -0.25; //-0.4
		primitive_7.delta_theta = -25.0 * (M_PI/180.0); //-60.0

		Primitive primitive_8;
		primitive_8.prim_id = 8;
		primitive_8.delta_x = 0.7;
		primitive_8.delta_y	= 0.25; //0.4
		primitive_8.delta_theta = 25.0 * (M_PI/180.0); //60.0


		primitive_list_.push_back(primitive_0);
		primitive_list_.push_back(primitive_1);
		primitive_list_.push_back(primitive_2);
		primitive_list_.push_back(primitive_3);
		primitive_list_.push_back(primitive_4);
		primitive_list_.push_back(primitive_5);
		primitive_list_.push_back(primitive_6);
		primitive_list_.push_back(primitive_7);
		primitive_list_.push_back(primitive_8);

		num_of_prim_ = primitive_list_.size();
	}


	void MotionPrimitives::applyPrim0(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_0 = primitive_list_[0];

		x_new = x + (primitive_0.delta_x * cos(theta));
		y_new = y + (primitive_0.delta_y * sin(theta));
		theta_new = theta;
	}

	void MotionPrimitives::applyPrim1(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_1 = primitive_list_[1];

		x_new = x - (primitive_1.delta_x * cos(theta));
		y_new = y - (primitive_1.delta_y * sin(theta));
		theta_new = theta;
	}

	void MotionPrimitives::applyPrim2(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_2 = primitive_list_[2];

		x_new = x + (primitive_2.delta_x * cos(theta));
		y_new = y + (primitive_2.delta_y * sin(theta));
		theta_new = theta;
	}

	void MotionPrimitives::applyPrim3(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_3 = primitive_list_[3];

		double r_d = sqrt( primitive_3.delta_x*primitive_3.delta_x + primitive_3.delta_y*primitive_3.delta_y );
		double theta_d = theta - atan( abs(primitive_3.delta_y / primitive_3.delta_x) );

		x_new = x + (r_d * cos(theta_d)); 
		y_new = y + (r_d * sin(theta_d));
		theta_new = theta + primitive_3.delta_theta;
	}

	void MotionPrimitives::applyPrim4(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_4 = primitive_list_[4];

		double r_d = sqrt( primitive_4.delta_x*primitive_4.delta_x + primitive_4.delta_y*primitive_4.delta_y );
		double theta_d = theta + atan( abs(primitive_4.delta_y / primitive_4.delta_x) );

		x_new = x + (r_d * cos(theta_d)); 
		y_new = y + (r_d * sin(theta_d));
		theta_new = theta + primitive_4.delta_theta;
	}

	void MotionPrimitives::applyPrim5(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_5 = primitive_list_[5];

		double r_d = sqrt( primitive_5.delta_x*primitive_5.delta_x + primitive_5.delta_y*primitive_5.delta_y );
		double theta_d = theta - atan( abs(primitive_5.delta_y / primitive_5.delta_x) );

		x_new = x + (r_d * cos(theta_d)); 
		y_new = y + (r_d * sin(theta_d));
		theta_new = theta + primitive_5.delta_theta;
	}

	void MotionPrimitives::applyPrim6(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_6 = primitive_list_[6];

		double r_d = sqrt( primitive_6.delta_x*primitive_6.delta_x + primitive_6.delta_y*primitive_6.delta_y );
		double theta_d = theta + atan( abs(primitive_6.delta_y / primitive_6.delta_x) );

		x_new = x + (r_d * cos(theta_d)); 
		y_new = y + (r_d * sin(theta_d));
		theta_new = theta + primitive_6.delta_theta;
	}

	void MotionPrimitives::applyPrim7(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_7 = primitive_list_[7];

		double r_d = sqrt( primitive_7.delta_x*primitive_7.delta_x + primitive_7.delta_y*primitive_7.delta_y );
		double theta_d = theta - atan( abs(primitive_7.delta_y / primitive_7.delta_x) );

		x_new = x + (r_d * cos(theta_d)); 
		y_new = y + (r_d * sin(theta_d));
		theta_new = theta + primitive_7.delta_theta;
	}

	void MotionPrimitives::applyPrim8(double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{
		Primitive primitive_8 = primitive_list_[8];

		double r_d = sqrt( primitive_8.delta_x*primitive_8.delta_x + primitive_8.delta_y*primitive_8.delta_y );
		double theta_d = theta + atan( abs(primitive_8.delta_y / primitive_8.delta_x) );

		x_new = x + (r_d * cos(theta_d)); 
		y_new = y + (r_d * sin(theta_d));
		theta_new = theta + primitive_8.delta_theta;
	}


	void MotionPrimitives::applyPrimitive(int prim_id, double x, double y, double theta, double& x_new, double& y_new, double& theta_new)
	{

		switch (prim_id)
		{

			case 0:
			{
				applyPrim0(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 1:
			{
				applyPrim1(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 2:
			{
				applyPrim2(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 3:
			{
				applyPrim3(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 4:
			{
				applyPrim4(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 5:
			{
				applyPrim5(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 6:
			{
				applyPrim6(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 7:
			{
				applyPrim7(x, y, theta, x_new, y_new, theta_new);
				break;
			}

			case 8:
			{
				applyPrim8(x, y, theta, x_new, y_new, theta_new);
				break;
			}
		}

	}





} // end namespace ak_planner