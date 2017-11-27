/*
 * Author: 	Abdul Moeed Zafar
 * Date:	1 Nov, 2017
 *
 * File:	moveit_interface.h
 */

#ifndef _MOVEIT_INTERFACE_H_
#define _MOVEIT_INTERFACE_H_

#include <boost/shared_ptr.hpp>

#include "ros/ros.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometry_msgs/Pose.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

#include <ak_planner/ak_planner_shared_variables.h>


namespace ak_planner
{

	class MoveitInterface
	{

	private:

		ros::NodeHandle node_handle_;
		
		robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
		planning_scene::PlanningScenePtr planning_scene_;

		ros::Publisher planning_scene_publisher_;
		ros::Publisher robot_state_publisher_;

		ros::Publisher start_state_publisher_;
		ros::Publisher goal_state_publisher_;
		

		int env_object_count_;

	public:

		MoveitInterface();
		void setRobotVirtualJoint(double x, double y, double theta);	
		void displayRobotTrajectory(std::vector<ak_planner_shared_variables::RoverTrajectoryNode>& traj);
		void addEnvObjectToPlanningScene(double position_x, double position_y, double position_z, double dimension_x, 
			double dimension_y, double dimension_z);		
		bool isStateInCollision(double x, double y, double theta);
		void printRobotCurrentVirtualJointState();
		void displayRobotState();
		void pause();

		void displayRobotStartState(double x, double y, double theta);
		void displayRobotGoalState(double x, double y, double theta);


	};

	typedef boost::shared_ptr<MoveitInterface> MoveitInterfacePtr;


} // end namespace ak_planner

#endif