/*
 * Author: 	Abdul Moeed Zafar
 * Date:	1 Nov, 2017
 *
 * File:	moveit_interface.cpp
 */


#include <ak_planner/moveit_interface.h>
#include <sstream>

namespace ak_planner
{

	MoveitInterface::MoveitInterface() : node_handle_("~")
	{
		ros::WallDuration sleep_t(2);
		planning_scene_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 5);
		while(planning_scene_publisher_.getNumSubscribers() < 1)
		{
			std::cout << "No /planning_scene subscribers. Waiting." << std::endl;
			sleep_t.sleep();
		}

		robot_state_publisher_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/robot_states", 5);

		robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
		
		planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));
		planning_scene_monitor_->startWorldGeometryMonitor();
		planning_scene_monitor_->startSceneMonitor();
		planning_scene_monitor_->startStateMonitor();

		planning_scene_ = planning_scene_monitor_->getPlanningScene();

		env_object_count_ = 0;
	}


	void MoveitInterface::setRobotVirtualJoint(double x, double y, double theta)
	{
		robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();

		robot_state.setVariablePosition("virtual_joint/x", x);
		robot_state.setVariablePosition("virtual_joint/y", y);
		robot_state.setVariablePosition("virtual_joint/theta", theta);

		planning_scene_->setCurrentState(robot_state);
	}	

	void MoveitInterface::displayRobotTrajectory(std::vector<ak_planner_shared_variables::RoverTrajectoryNode>& traj)
	{
		ros::Rate loop_rate(1);

		for(std::vector<ak_planner_shared_variables::RoverTrajectoryNode>::iterator it = traj.begin() ; it < traj.end() ; it++)
		{
			robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
			robot_state.setVariablePosition("virtual_joint/x", it->x);
			robot_state.setVariablePosition("virtual_joint/y", it->y);
			robot_state.setVariablePosition("virtual_joint/theta", it->theta);

			moveit_msgs::DisplayRobotState robot_state_msg;
			robot_state::robotStateToRobotStateMsg(robot_state, robot_state_msg.state);
			
			while(robot_state_publisher_.getNumSubscribers() < 1)
			{
				ROS_WARN_ONCE("Waiting for subscribers for /robot_states topic.");
			}

			robot_state_publisher_.publish(robot_state_msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	void MoveitInterface::addEnvObjectToPlanningScene(double position_x, double position_y, double position_z, 
			double dimension_x, double dimension_y, double dimension_z)
	{
		moveit_msgs::PlanningScene scene_object;

		moveit_msgs::AttachedCollisionObject attached_object;
		attached_object.object.header.frame_id = "odom_combined";
		
		std::string object_id = "box";
		std::stringstream ss;
		ss << env_object_count_;
		object_id = object_id + ss.str();
		attached_object.object.id = object_id;
		env_object_count_++;

		geometry_msgs::Pose object_pose;
		object_pose.position.x = position_x;
		object_pose.position.y = position_y;
		object_pose.position.z = position_z;
		object_pose.orientation.w = 1.0;

		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[0] = dimension_x;
		primitive.dimensions[1] = dimension_y;
		primitive.dimensions[2] = dimension_z;
		
		attached_object.object.primitives.push_back(primitive);
		attached_object.object.primitive_poses.push_back(object_pose);
		attached_object.object.operation = attached_object.object.ADD;

		scene_object.world.collision_objects.push_back(attached_object.object);
		scene_object.is_diff = true;

		while(planning_scene_publisher_.getNumSubscribers() < 1)
		{
			ROS_WARN_ONCE("Waiting for subscribers for /planning_scene topic.");
		}

		std::cout << "Subscribed!" << std::endl;

		planning_scene_publisher_.publish(scene_object);
		
		ros::spinOnce();
		ros::WallDuration sleep_t(1);
		sleep_t.sleep();

		bool obj_exist = planning_scene_->getWorld()->hasObject(object_id);
		std::cout << "Object " << object_id << " exists in collision world: " << obj_exist << std::endl;

	}


	bool MoveitInterface::isStateInCollision(double x, double y, double theta)
	{
		robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
		robot_state.setVariablePosition("virtual_joint/x", x);
		robot_state.setVariablePosition("virtual_joint/y", y);
		robot_state.setVariablePosition("virtual_joint/theta", theta);

		planning_scene_->setCurrentState(robot_state);

		collision_detection::CollisionRequest collision_request;
		collision_detection::CollisionResult collision_result;
		collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();

		planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm);

		if(collision_result.collision)
			return true;
		else
			return false;
	}

	void MoveitInterface::printRobotCurrentVirtualJointState()
	{
		robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();

		std::cout << "Robot State: " << std::endl;
		std::cout << "virutal_joint/x: " << robot_state.getVariablePosition("virtual_joint/x") << std::endl;
		std::cout << "virutal_joint/y: " << robot_state.getVariablePosition("virtual_joint/y") << std::endl;
		std::cout << "virutal_joint/theta: " << robot_state.getVariablePosition("virtual_joint/theta") << std::endl;
	}

	void MoveitInterface::displayRobotState()
	{
		ros::Rate loop_rate(0.5);
		bool exit_display = false;

		while(!exit_display)
		{


			robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();

			moveit_msgs::DisplayRobotState robot_state_msg;
			robot_state::robotStateToRobotStateMsg(robot_state, robot_state_msg.state);
			while(robot_state_publisher_.getNumSubscribers() < 1)
			{
				ROS_WARN_ONCE("Waiting for subscribers for /robot_states topic.");
			}

			robot_state_publisher_.publish(robot_state_msg);
			// ros::spinOnce();

			// char c;
			// std::cout << "Enter Y/y to exit display: ";
			// std::cin >> c;
			// if(c == 'Y' || c == 'y')
			// {
			// 	exit_display = true;
			// }
			exit_display = true;

			loop_rate.sleep();
		}
	}

	void MoveitInterface::pause()
	{
		char c;
		std::cout << "Enter any key to continue. ";
		std::cin >> c;
	}
		




} //end namespace ak_planner