/*
 * Author: 	Abdul Moeed Zafar
 * Date:	5 Nov, 2017
 *
 * File:	occupancy_grid_2d.h
 */

#ifndef _OCCUPANCY_GRID_2D_H_
#define _OCCUPANCY_GRID_2D_H_


#include <moveit/distance_field/propagation_distance_field.h>
#include <ros/package.h>
#include <boost/shared_ptr.hpp>


namespace ak_planner
{
	
	class OccupancyGrid2D
	{

	private:

		boost::shared_ptr<distance_field::PropagationDistanceField> distance_field_;


	public:

		double origin_x_;
		double origin_y_;
		double size_x_;
		double size_y_;

		OccupancyGrid2D(double size_x, double size_y, double origin_x, double origin_y, double resolution);
		~OccupancyGrid2D();

		inline void gridToWorld(int gx, int gy, double& wx, double& wy);
		inline void worldToGrid(double wx, double wy, int& gx, int& gy);
		inline double getDistanceFromGridCoordinates(int gx, int gy);
		inline double getDistanceFromWorldCoordinates(double wx, double wy);
		inline void getWorldSize(double& dim_x, double& dim_y);
		inline void getGridSize(int& dim_x, int& dim_y);
		inline double getResolution();

		void addObjectToGrid(const std::vector<std::vector<double> >& object_primitive_coords_2d); 	// Parametrize with EnvObstacles
		void addObstacle(double size_x, double size_y, double origin_x, double origin_y);
		void addPointsToField(const std::vector<Eigen::Vector3d>& points);

	};

	typedef boost::shared_ptr<OccupancyGrid2D> OccupancyGrid2DPtr;



	inline void OccupancyGrid2D::gridToWorld(int gx, int gy, double& wx, double& wy)
	{
		double dummy;
		distance_field_->gridToWorld(gx, gy, 0, wx, wy, dummy);
	}

	inline void OccupancyGrid2D::worldToGrid(double wx, double wy, int& gx, int& gy)
	{
		int dummy;
		distance_field_->worldToGrid(wx, wy, 0.0, gx, gy, dummy);
	}

	inline double OccupancyGrid2D::getDistanceFromGridCoordinates(int gx, int gy)
	{
		return distance_field_->getDistance(gx, gy, 0);
	}

	inline double OccupancyGrid2D::getDistanceFromWorldCoordinates(double wx, double wy)
	{
		int gx, gy;
		worldToGrid(wx, wy, gx, gy);
		return distance_field_->getDistance(gx, gy, 0);
	}

	inline void OccupancyGrid2D::getWorldSize(double& dim_x, double& dim_y)
	{
		dim_x = distance_field_->getSizeX();
		dim_y = distance_field_->getSizeY();
	}

	inline void OccupancyGrid2D::getGridSize(int& dim_x, int& dim_y)
	{
		dim_x = distance_field_->getSizeX() / distance_field_->getResolution();
		dim_y = distance_field_->getSizeY() / distance_field_->getResolution();
	}

	inline double OccupancyGrid2D::getResolution()
	{
		return distance_field_->getResolution();
	}


} // end namespace ak_planner



#endif