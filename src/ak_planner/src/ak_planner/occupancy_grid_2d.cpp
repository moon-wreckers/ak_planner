/*
 * Author: 	Abdul Moeed Zafar
 * Date:	5 Nov, 2017
 *
 * File:	occupancy_grid_2d.cpp
 */


#include <ak_planner/occupancy_grid_2d.h>

namespace ak_planner
{

	OccupancyGrid2D::OccupancyGrid2D(double size_x, double size_y, double origin_x, double origin_y, double resolution) : 
		distance_field_(new distance_field::PropagationDistanceField(size_x, size_y, resolution, resolution, origin_x, origin_y, 0.0, 10.0)) 
	{
		distance_field_->reset();
		
		origin_x_ = origin_x;
		origin_y_ = origin_y;
		
		size_x_ = size_x;
		size_y_ = size_y;
	}

	OccupancyGrid2D::~OccupancyGrid2D()
	{

	}

	void OccupancyGrid2D::addObjectToGrid(const std::vector<std::vector<double> >& object_primitive_coords_2d)
	{
		// std::vector<std::vector<double> > object_primitive_coords_2d;

		// std::vector<double> test_object_coords;
		// test_object_coords.push_back(0.5);
		// test_object_coords.push_back(0.5);
		// test_object_coords.push_back(0.2);
		// test_object_coords.push_back(0.2);
		// object_primitive_coords_2d.push_back(test_object_coords);

		for(int i=0 ; i<((int)object_primitive_coords_2d.size()) ; i++)
		{
			assert(object_primitive_coords_2d[i].size() == 4);

			addObstacle(object_primitive_coords_2d[i][2], object_primitive_coords_2d[i][3], object_primitive_coords_2d[i][0], object_primitive_coords_2d[i][1]);
		}
	}

	void OccupancyGrid2D::addObstacle(double size_x, double size_y, double origin_x, double origin_y)
	{
		int num_points = 0;
		std::vector<Eigen::Vector3d> pts;

		for(double x = origin_x - size_x/2.0 ; x <= origin_x + size_x/2.0 ; x += distance_field_->getResolution())
		{
			for(double y = origin_y - size_y/2.0 ; y <= origin_y + size_y/2.0 ; y += distance_field_->getResolution())
			{
				pts.push_back(Eigen::Vector3d(x, y, 0.0));
				//std::cout << "add obstacle: x: " << x << " y: " << y << std::endl;
			}
		}

		addPointsToField(pts);
	}

	void OccupancyGrid2D::addPointsToField(const std::vector<Eigen::Vector3d>& points)
	{
		EigenSTL::vector_Vector3d pts(points.size());
		for(size_t i=0 ; i<points.size() ; ++i)
		{
			pts[i] = Eigen::Vector3d(points[i].x(), points[i].y(), points[i].z());
		}

		distance_field_->addPointsToField(pts);
	}





} // end namespace ak_planner