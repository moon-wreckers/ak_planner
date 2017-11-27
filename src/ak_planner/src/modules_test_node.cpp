/*
 * Author: 	Abdul Moeed Zafar
 * Date:	1 Nov, 2017
 *
 * File:	modules_test_node.cpp
 */

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <ak_planner/moveit_interface.h>
#include <ak_planner/ak_planner_shared_variables.h>

#include <ak_planner/bfs_2d_heuristic.h>
#include <ak_planner/occupancy_grid_2d.h>
#include <ak_planner/graph_state_manager.h>
#include <ak_planner/graph_state.h> 
#include <ak_planner/dubins_heuristic.h>

#include <ak_planner/weighted_A_star_planner.h>

#include <algorithm>



#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <ak_planner/dubins_distance.h>

#include <ak_simulation/rover_simulator.h>


#include <ros/package.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/regex.hpp>


#include <ak_planner/motion_primitives_parser.h>


using namespace ak_planner;


struct CompareByFValue 
{
	bool operator()(const ak_planner::GraphState& a, const ak_planner::GraphState& b)
	{
		return a.getFVal() < b.getFVal();
	}
} compareByFVal;


int getPrimitiveDirectionforRobotPose(double angle)
{
	int NUMOFDIRS = 36;
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == NUMOFDIRS) {
        dir = 0;
    }
    return dir;
}



//--------------------------------------- Visualizing Trajectory--------------------------------------------------------


void generateTrajectoryPointsFromPath(GraphStateList solution_path, std::vector<std::vector<double> >& traj_points)
{
	for(int i=0 ; i<solution_path.size() ; i++)
	{
		std::vector<double> traj_single_point;
		traj_single_point = solution_path[i].getStateVariables();

		traj_points.push_back(traj_single_point);

		//TODO: Interpolation between two points using dubins distance.
	}

}

void interpolateTrajectory(std::vector<std::vector<double> >& traj_points, std::vector<std::vector<double> >& interpolated_traj_points)
{



	double turning_radius = ak_planner_shared_variables::rover_min_turning_radius;
	double dubins_resolution = ak_planner_shared_variables::dubins_path_resolution;

	for(int i=0 ; i<traj_points.size()-1 ; i++)
	{

		double q0[3];
		q0[0] = traj_points[i].at(0);
		q0[1] = traj_points[i].at(1);
		q0[2] = traj_points[i].at(2);

		double q1[3];
		q1[0] = traj_points[i+1].at(0);
		q1[1] = traj_points[i+1].at(1);
		q1[2] = traj_points[i+1].at(2);

		dubins_distance::DubinsPath path;
	    dubins_distance::dubins_init( q0, q1, turning_radius, &path);
    	//dubins_distance::dubins_path_sample_many( &path, printConfiguration, dubins_resolution, NULL);

	    std::vector<std::vector<double> > dubins_path_vector;
	    dubins_distance::getDubinsPath(&path, dubins_resolution, dubins_path_vector);
	    std::cout << "Segment: " << i << "  dubins_path_vector size: " << dubins_path_vector.size() << std::endl;

	    interpolated_traj_points.insert(interpolated_traj_points.end(), dubins_path_vector.begin(), dubins_path_vector.end());

	}


	interpolated_traj_points.push_back(traj_points.back());
	std::cout << "Size of interpolated path: " << interpolated_traj_points.size() << std::endl;
}



//----------------------------------------------------------------------------------------------------------------------


int main (int argc, char** argv)
{
	ros::init(argc, argv, "modules_test_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Duration sleep_time(2);
	sleep_time.sleep();


	//------------------------------------------ Testing File Parser ---------------------------------------------------

	// std::cout << "Begining testing File parser" << std::endl;



	// MotionPrimitivesParser motion_primitive_parser;

	// motion_primitive_parser.readPrimitives();

	// std::vector<TaskSpacePrimitive> primitives_list_0 = motion_primitive_parser.getPrimitiveListFromStartAngleID(15);

	// for(int i=0 ; i<primitives_list_0.size() ; i++)
	// {
	// 	primitives_list_0[i].printPrimitive();
	// 	std::cout << std::endl;
	// }
	// std::cout << "Primitive List 0 size: " << primitives_list_0.size() << std::endl;

	// // for(double i=0 ; i<6.28318 ; i+=0.1)
	// // {

	// // 	std::cout << "Discretizing angle: " << i << "\t Discretized: " << motion_primitive_parser.discretizeAngle(i) << std::endl;

	// // }
	// // motion_primitive_parser.printPrimitives();


	// std::cout << "Ending testing File parser" << std::endl << std::endl;

	// return 0;

	//------------------------------------------ Testing Planner -------------------------------------------------------

	double oc_size_x = 10.0;
	double oc_size_y = 10.0; 
	double oc_origin_x = -1.0;
	double oc_origin_y = -1.0;
	double oc_resolution = 0.025;

	double bfs_goal_region_radius = 0.1; 
	double bfs_weight_multiplier = 1; 
	double bfs_obstacle_inflation = 0.6; 

	double d_weight_multiplier = 2;
	
	std::vector<std::vector<double> > object_primitive_coords_3d;
	std::vector<double> obstacle1;
	obstacle1.resize(6);
	obstacle1[0] = 3; //1  //origin x
	obstacle1[1] = 2; //1  //origin y
	obstacle1[2] = 0.5;    //origin z
	obstacle1[3] = 1.5;    //size x
	obstacle1[4] = 1.5;    //size y
	obstacle1[5] = 1;      //size z
	object_primitive_coords_3d.push_back(obstacle1);
	 
	double heuristic_weight_multiplier = 10;

	double start_state_x = 0.0;
	double start_state_y = 2.0;
	double start_state_theta = (0.0 * (M_PI/180.0));

	double goal_state_x = 6;
	double goal_state_y = 2.0;
	double goal_state_theta = (30.0 * (M_PI/180.0)); //TODO: Cater for entering negative angles.

	bool verbosity = true;

	WeightedAStarPlanner wA_planner(oc_size_x, oc_size_y, oc_origin_x, oc_origin_y, oc_resolution, 
		bfs_goal_region_radius, bfs_weight_multiplier, bfs_obstacle_inflation, d_weight_multiplier, object_primitive_coords_3d, 
		heuristic_weight_multiplier);

	//Simulator
	rover_simulation::RoverSimulator rover_simulator;


	wA_planner.plan(start_state_x, start_state_y, start_state_theta, goal_state_x, goal_state_y, goal_state_theta, verbosity);

	// wA_planner.printSolutionPathStates();



	std::vector<std::vector<double> > path_trajectory;
	generateTrajectoryPointsFromPath(wA_planner.solution_path_, path_trajectory);


	//Interpolating Trajectory
	std::vector<std::vector<double> > interpolated_path_trajectory;
	// interpolated_path_trajectory = path_trajectory;
	wA_planner.getInterpolatedSolutionPath(interpolated_path_trajectory);


	//DEBUGGING INTERPOLATED PATH
	std::cout << "interpolated_path_trajectory size: " << interpolated_path_trajectory.size() << std::endl;
	// for(int i=0 ; i<interpolated_path_trajectory.size() ; i++)
	// {	
	// 	std::vector<double> pose = interpolated_path_trajectory[i];

	// 	std::cout << pose[0] << "\t" << pose[1] << "\t" << pose[2] << std::endl;

	// 	if( (i % 50) == 0)
	// 	{
	// 		char c;
	// 		std::cin >> c;
	// 	}
	// }




	// geometry_msgs::PoseArray path_trajectory_pose_array;
	// for(int i = 0 ; i < path_trajectory.size() ; i++)
	// {
	// 	geometry_msgs::Pose rover_pose;
	// 	rover_pose.position.x = path_trajectory[i].at(0);
	// 	rover_pose.position.y = path_trajectory[i].at(1);
	// 	rover_pose.position.z = 0;
	// 	rover_pose.orientation = tf::createQuaternionMsgFromYaw( path_trajectory[i].at(2) );

	// 	path_trajectory_pose_array.poses.push_back(rover_pose);		
	// }

	geometry_msgs::PoseArray path_trajectory_pose_array;
	for(int i = 0 ; i < interpolated_path_trajectory.size() ; i++)
	{
		geometry_msgs::Pose rover_pose;
		rover_pose.position.x = interpolated_path_trajectory[i].at(0);
		rover_pose.position.y = interpolated_path_trajectory[i].at(1);
		rover_pose.position.z = 0;
		rover_pose.orientation = tf::createQuaternionMsgFromYaw( interpolated_path_trajectory[i].at(2) );

		path_trajectory_pose_array.poses.push_back(rover_pose);		
	}




	// Simulation:
	
	ros::Publisher trajectory_publisher = nh.advertise<geometry_msgs::PoseArray>("/planned_trajectory_topic", 100);
	trajectory_publisher.publish(path_trajectory_pose_array);


	while(ros::ok())
	{
		char command;
		std::cout << "Enter Y to play trajectory. Press any key to finish.  ";
		std::cin >> command;

		if(command == 'Y' || command == 'y')
		{
			// trajectory_publisher.publish(trajectory1);
			ros::spinOnce();
			rover_simulator.runSimulation();
			//rover_simulator.publishTrajectory();
		} 
		else
		{
			break;
		}

	}

	




	//------------------------------------------ Testing compareByFVal -------------------------------------------------

	// GraphStateList g_list;
	// std::sort(g_list.begin(), g_list.end(), compareByFVal);



	//------------------------------------------ Testing Discretizing Angle Value --------------------------------------


	// for(double angle = 0 ; angle <= 360 ; angle=angle+1)
	// {
	// 	int dir = getPrimitiveDirectionforRobotPose(angle * (M_PI/180.0));
	// 	//double angle_deg = angle * (180.0/M_PI);

	// 	std::cout << "Degrees: " << angle << "\tDirection: " << dir << std::endl;
	// }


	//------------------------------------------ graph_state_manager Testing -------------------------------------------

	// std::cout << "Graph State Manager Testing Starting. " << std::endl;

	// GraphStateManagerPtr graph_manager_ = boost::make_shared<GraphStateManager>();

	// GraphState start_state;
	// start_state.setStateVariables(0, 0, 0);

	// std::cout << "Here 1" << std::endl;

	// GraphState goal_state;
	// goal_state.setStateVariables(4, 4, -3.142/2);

	// graph_manager_->addStartState(start_state);
	// graph_manager_->addGoalState(goal_state);


	// GraphState state1;
	// state1.setPrimitiveId(2);
	// state1.setPredId(1);
	// state1.setStateType(GraphState::SINGLE_POINT);
	// state1.setStateVariables(1, 1, 0.2);
	// graph_manager_->addGraphState(state1);

	// GraphState state2;
	// state2.setPrimitiveId(3);
	// state2.setPredId(1);
	// state2.setStateType(GraphState::SINGLE_POINT);
	// state2.setStateVariables(1, -1, 0.2);
	// graph_manager_->addGraphState(state2);


	// graph_manager_->printIDGraphStateMap();


	// graph_manager_->deleteGraphState(state1);
	// graph_manager_->printIDGraphStateMap();



	// std::cout << std::endl;

	// GraphState state_returned = graph_manager_->getGraphState(2);
	// if(!state_returned.isValid())
	// {
	// 	std::cout << "State does not exist in Graph State Manager list. Dummy State returned." << std::endl;
	// }
	// else
	// {
	// 	std::cout << "Valid State returned." << std::endl;
	// 	state_returned.printStateAttributes();
	// }

	// std::cout << std::endl;

	// state_returned = graph_manager_->getGraphState(3);
	// if(!state_returned.isValid())
	// {
	// 	std::cout << "State does not exist in Graph State Manager list. Dummy State returned." << std::endl;
	// }
	// else
	// {
	// 	std::cout << "Valid State returned." << std::endl;
	// 	state_returned.printStateAttributes();
	// }

	// std::cout << std::endl;


	// int id_returned = graph_manager_->getStateId(state1);
	// if(id_returned == GraphStateManager::STATE_NONEXISTENT)
	// {
	// 	std::cout << "State non-existent." << std::endl;
	// }
	// else
	// {
	// 	std::cout << "id_returned: " << id_returned << std::endl;
	// }

	// std::cout << std::endl;
	
	// id_returned = graph_manager_->getStateId(state2);
	// if(id_returned == GraphStateManager::STATE_NONEXISTENT)
	// {
	// 	std::cout << "State non-existent." << std::endl;
	// }
	// else
	// {
	// 	std::cout << "id_returned: " << id_returned << std::endl;
	// }




	// std::cout << std::endl << "Graph State Manager Testing Ending. " << std::endl << std::endl;
	

	//------------------------------------------ dubins_heuristic Testing -----------------------------------------------------------------

	// std::cout << "Starting dubins_heuristic Testing." << std::endl << std::endl;


	// int weight_multiplier = 1;


	// std::cout << "Dubins distance direct calculation: " << std::endl;
	// double q0[] = { 0,0,0 };
 //    double q1[] = { 1,-1,-3.142/2 };
 //    double turning_radius = 0.5;
 //    dubins_distance::DubinsPath path;
 //    dubins_distance::dubins_init( q0, q1, turning_radius, &path);
 //    dubins_distance::printDubinsPath( &path, 0.1);
 //    double path_cost = dubins_path_length(&path) / 0.1 * weight_multiplier;
 //    std::cout << "Path cost: " << path_cost << std::endl;

    
 //    std::cout << std::endl;

    
 //    std::cout << "Dubins heuristic calculation: " << std::endl;

	// DubinsHeuristicPtr dubins_heuristic_ = boost::make_shared<DubinsHeuristic>(weight_multiplier);
	
	// std::vector<double> goal_state;
 //    goal_state.resize(3);
 //    goal_state[0] = q1[0];
 //    goal_state[1] = q1[1];
 //    goal_state[2] = q1[2];
 //    dubins_heuristic_->setGoal(goal_state);

 //    GraphState test_state;
 //    test_state.setStateVariables(q0[0], q0[1], q0[2]);
 //    int h_value = dubins_heuristic_->getGoalHeuristic(test_state);
 //    std::cout << "Path cost: " << h_value << std::endl;



	// std::cout << std::endl << "Ending dubins_heuristic Testing. " << std::endl << std::endl;

	
	//------------------------------------------ bfs_2d_heuristic Testing -----------------------------------------------------------------

	// std::cout << "Starting bfs_2d_heuristic test:" << std::endl;

	// double size_x = 10.0;
	// double size_y = 10.0;
	// double origin_x = -1;
	// double origin_y = -1;
	// double resolution = 0.1;

	// std::vector<std::vector<double> > obstacles;
	// std::vector<double> obstacle1;
	// obstacle1.resize(4);
	// obstacle1[0] = 0.5;
	// obstacle1[1] = 0.5;
	// obstacle1[2] = 0.2;
	// obstacle1[3] = 0.2;
	// obstacles.push_back(obstacle1);
	
	
	// OccupancyGrid2DPtr occupancy_grid_2d_ = boost::make_shared<OccupancyGrid2D>(size_x, size_y, origin_x, origin_y, resolution);
	// occupancy_grid_2d_->addObjectToGrid(obstacles);

	// double goal_region_radius = 0.01;
	// double bfs_weight_multiplier = 1.0;
	// double obstacle_inflation = 0.1;

	// BFS2DHeuristicPtr bfs_2d_heuristic_ = boost::make_shared<BFS2DHeuristic>(goal_region_radius, bfs_weight_multiplier, occupancy_grid_2d_, obstacle_inflation);
	// bfs_2d_heuristic_->updateBFS2DGrid();

	// std::vector<double> goal_pose;
	// goal_pose.push_back(0.0);
	// goal_pose.push_back(0.0);
	// goal_pose.push_back(0.0);
	// bfs_2d_heuristic_->setGoal(goal_pose);

	// bfs_2d_heuristic_->printBFS2DHeuristicMap();


	// std::cout << std::endl;

	// GraphState test_state;
	// test_state.setStateVariables(1, 1, 0);
	// double heur_val = bfs_2d_heuristic_->getGoalHeuristic(test_state);
	// std::cout << "bfs heur val: " << heur_val << std::endl;


	// std::cout << "Ending bfs_2d_heuristic test." << std::endl;




	//------------------------------------------ moveit_interface Testings-------------------------------------



	// ak_planner::MoveitInterface moveit_interface;

	// /*moveit_interface.addEnvObjectToPlanningScene(0.5, 0, 0, 1, 1, 1);*/

	// moveit_interface.addEnvObjectToPlanningScene(2, 2, 0, 1, 1, 1);
	
	// /*if(moveit_interface.isStateInCollision(0, 0, 0))
	// {
	// 	std::cout << "In collision" << std::endl;
	// }
	// else
	// {
	// 	std::cout << "Not in collision" << std::endl;
	// }*/
	
	// // moveit_interface.setRobotVirtualJoint(2.0, 1.0, ((M_PI / 180.0) * 45) );
	

	// /* ros::Duration sleep_time2(0.01);

	// sleep_time2.sleep(); */

	// if(moveit_interface.isStateInCollision(2.0, 1.0, ((M_PI / 180.0) * 45) ) )
	// {
	// 	std::cout << "In collision" << std::endl;
	// }
	// else
	// {
	// 	std::cout << "Not in collision" << std::endl;
	// }
	// // moveit_interface.displayRobotState();

	// /* sleep_time2.sleep(); */	
	
	// if(moveit_interface.isStateInCollision(2.0, 0.0, ((M_PI / 180.0) * 45) ) )
	// {
	// 	std::cout << "In collision" << std::endl;
	// }
	// else
	// {
	// 	std::cout << "Not in collision" << std::endl;
	// }
	// moveit_interface.displayRobotState();

	//--------------------------------------------------------------------------------------------------------



	std::cout << "End of Node" << std::endl;



	// while(ros::ok())
	// {}


	return 0;
}