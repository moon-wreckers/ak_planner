/*
 * Author: 	Abdul Moeed Zafar
 * Date:	7 Nov, 2017
 *
 * File:	BFS_2D.cpp
 */


 #include <ak_planner/BFS_2D.h>

 namespace ak_planner
 {

 	#define EXPAND_NEIGHBOR_2D(offset)										\
 		if(distance_grid[current_node + offset] < 0)						\
 		{																	\
 			queue[queue_tail++] = current_node + offset;					\
 			distance_grid[current_node + offset] = current_cost;			\
 		}																	


	const double BFS_2D::WALL = std::numeric_limits<float>::max()-1000;
	const double BFS_2D::UNDISCOVERED = -1;



	BFS_2D::BFS_2D(int width, int length)
	{
		if (width <= 0 || length <= 0)
		{
			std::cout << "Invalid width and length of BFS_2D grid specified. " << std::endl;
			throw 0;
		}

		dim_x = width + 2;
		dim_y = width + 2;

		dim_xy = dim_x * dim_y;

		distance_grid = new double[dim_xy];
		queue = new int[width*length];

		for(int node=0 ; node<dim_xy ; node++)
		{
			int x = node % dim_x;
			int y = node / dim_x % dim_y;

			if(x == 0 || x == dim_x-1 || y == 0 || y == dim_y-1)
			{
				distance_grid[node] = WALL;
			}
			else
			{
				distance_grid[node] = UNDISCOVERED;
			}
		}

		running = false;
	}


	BFS_2D::~BFS_2D()
	{
		delete[] distance_grid;
		delete[] queue;
	}


	void BFS_2D::getDimensions(int& width, int& length)
	{
		width = dim_x - 2;
		length = dim_y - 2;
	}

	void BFS_2D::setWall(int x, int y) 
    {
    	if (running) 
    	{
        	std::cout << "Cannot modify the grid while the search is running" << std::endl;
        	return;
    	}

    	int node = getNode(x, y);
    	distance_grid[node] = WALL;
	}

	bool BFS_2D::isWall(int x, int y)
	{
		int node = getNode(x, y);
		return distance_grid[node] == WALL;
	}


	void BFS_2D::run(int x, int y)
	{
		if(running)
		{
			return;
		}

		for(int i=0 ; i < dim_xy ; i++)
		{
			if(distance_grid[i] != WALL)
			{
				distance_grid[i] = UNDISCOVERED;
			}
		}

		origin = getNode(x, y);

		queue_head = 0;
		queue_tail = 1;
		queue[0] = origin;

		distance_grid[origin] = 0;
		running = true;
		search(dim_x, distance_grid, queue, queue_head, queue_tail);
		running = false;
	}

	void BFS_2D::search(int width, double volatile* distance_grid, int* queue, int& queue_head, int& queue_tail)
	{

		while(queue_head < queue_tail)
		{
			int current_node = queue[queue_head++];

			double current_cost;

			current_cost = distance_grid[current_node] + 1;

			EXPAND_NEIGHBOR_2D(-width);
            EXPAND_NEIGHBOR_2D(1);
            EXPAND_NEIGHBOR_2D(width);
            EXPAND_NEIGHBOR_2D(-1);

            current_cost = distance_grid[current_node] + sqrt(2);

            EXPAND_NEIGHBOR_2D(-width-1);
            EXPAND_NEIGHBOR_2D(-width+1);
            EXPAND_NEIGHBOR_2D(width+1);
            EXPAND_NEIGHBOR_2D(width-1);
		}

		std::cout << "BFS2D search complete." << std::endl;

	}


	double BFS_2D::getDistance(int x, int y)
	{
		int node = getNode(x, y);

		while(running);
		if(distance_grid[node] < 0)
		{
			std::cout << "ERROR: distance_grid[node] is less than 0. It should have some value." << std::endl;
			throw 0;
		}

		return distance_grid[node];
	}



 } // end namespace ak_planner