/*
 * Author: 	Abdul Moeed Zafar
 * Date:	14 Nov, 2017
 *
 * File:	motion_primitives.h
 */

#ifndef _MOTION_PRIMITIVES_H_
#define _MOTION_PRIMITIVES_H_

#include <ros/ros.h>

namespace ak_planner
{

	class MotionPrimitives
	{

	private:

		struct Primitive
		{
			int prim_id;

			double delta_x;
			double delta_y;
			double delta_theta;

		};


		int num_of_prim_;
		std::vector<Primitive> primitive_list_;

		void applyPrim0(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim1(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim2(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim3(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim4(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim5(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim6(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim7(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);
		void applyPrim8(double x, double y, double theta, double& x_new, double& y_new, double& theta_new);


	public:

		MotionPrimitives();
		~MotionPrimitives();

		inline int getNumOfPrim() const;

		void initPrimitives();
		void populatePrimitives();

		void applyPrimitive(int prim_id, double x, double y, double theta, double& x_new, double& y_new, double& theta_new);

	};

	typedef boost::shared_ptr<MotionPrimitives> MotionPrimitivesPtr;

	inline int MotionPrimitives::getNumOfPrim() const
	{
		return num_of_prim_;
	}



} // end namespace ak_planner

#endif