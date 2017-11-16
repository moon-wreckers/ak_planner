/*
 * Author: 	Abdul Moeed Zafar
 * Date:	7 Nov, 2017
 *
 * File:	BFS_2D.h
 */

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <limits>

#ifndef _BFS_2D_H_
#define _BFS_2D_H_


namespace ak_planner
{

	class BFS_2D
	{

	private:

		int dim_x, dim_y;
		int dim_xy;
		int origin;
		double volatile* distance_grid;
		int* queue;
		int queue_head, queue_tail;
		volatile bool running;

		void search(int width, double volatile* distance_grid, int* queue, int& queue_head, int& queue_tail);
		inline int getNode(int x, int y);

	public:

		static const double WALL;
		static const double UNDISCOVERED;

		BFS_2D(int width, int length);
		~BFS_2D();
		
		void getDimensions(int& width, int& length);
		void setWall(int x, int y);
		bool isWall(int x, int y);
		void run(int x, int y);
		double getDistance(int x, int y);

	};

	

	typedef boost::shared_ptr<BFS_2D> BFS_2DPtr;

	
	inline int BFS_2D::getNode(int x, int y)
	{
		if (x < 0 || y < 0 || x >= dim_x - 2 || y >= dim_y - 2) 
        {
            std::cout << "Invalid coordinates given to get Node at (x,y) = (" << x << ", " << y << ")" << std::endl;
            throw std::runtime_error("Invalid coordinates for BFS2D grid");
        }

        return (y + 1) * dim_x + (x + 1);
	}


} // end namespace ak_planner


#endif