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

int main (int argc, char** argv)
{
	ros::init(argc, argv, "rover_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;


	//--------------------------------TESTING JOINT VALUES SET/GET IN URDF/ SRDF------------------------------------------------------------

	// ros::Duration sleep_time(10.0);
	// sleep_time.sleep();
	// sleep_time.sleep();

	// robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	// std::cout << "Model frame:" << kinematic_model->getModelFrame().c_str() << std::endl;

	// robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	// kinematic_state->setToDefaultValues();
	// const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("front_wheels");

	// const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();

	// std::vector<double> joint_values;
	// kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	// for(std::size_t i=0 ; i<joint_names.size() ; ++i)
	// {
	// 	ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	// }

	// joint_values[0] = 1.57;
	// kinematic_state->setJointGroupPositions(joint_model_group, joint_values);



	//----------------------------------------TESTING ADD OBJECTS TO ENVIRONMENT---------------------------------------------------------------------

	// ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	// while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	// {
	// 	ros::WallDuration sleep_t(0.5);
	// 	std::cout << "No subscribers!" << std::endl;
	// 	sleep_t.sleep();
	// }

	// std::cout << "Subscribed!" << std::endl;

	// moveit_msgs::AttachedCollisionObject attached_object;
	// attached_object.link_name = "base_link";
	// attached_object.object.header.frame_id = "base_link";
	// attached_object.object.id = "box";

	// geometry_msgs::Pose pose;
	// pose.position.x = 1;
	// pose.position.y = 1;
	// pose.position.z = 1;
	// pose.orientation.w = 1.0;
	
	// shape_msgs::SolidPrimitive primitive;
	// primitive.type = primitive.BOX;
	// primitive.dimensions.resize(3);
	// primitive.dimensions[0] = 1;
	// primitive.dimensions[1] = 1;
	// primitive.dimensions[2] = 1;

	// attached_object.object.primitives.push_back(primitive);
	// attached_object.object.primitive_poses.push_back(pose);
	// attached_object.object.operation = attached_object.object.ADD;

	// moveit_msgs::PlanningScene planning_scene;
	// planning_scene.world.collision_objects.push_back(attached_object.object);
	// planning_scene.is_diff = true;
	// planning_scene_diff_publisher.publish(planning_scene);
	
	// while(1)
	// {
	// 	sleep_time.sleep();
	// }


	//----------------------------------- TESTING COLLISION CHECKING-----------------------------------------------------------------------

	ros::Duration sleep_time(5.0);
	ros::WallDuration sleep_t(2);
	sleep_time.sleep();
	//sleep_time.sleep();

	ros::Publisher planning_scene_publisher_ = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 5); // Publisher for planning_scene

	while(planning_scene_publisher_.getNumSubscribers() < 1)
	{
		std::cout << "No subscribers!" << std::endl;
		sleep_t.sleep();
	}

	robot_model_loader::RobotModelLoaderPtr robot_model_loader; // Model Loader
	robot_model_loader.reset(new robot_model_loader::RobotModelLoader("robot_description")); // Loading URDF
	robot_model::RobotModelPtr kinematic_model = robot_model_loader->getModel(); //Getting Model

	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; // Planning Scene Monitor
	planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader)); //Reseting by initializing with model loader
	planning_scene_monitor_->startWorldGeometryMonitor(); //Monitor Started
	planning_scene_monitor_->startSceneMonitor();
	planning_scene_monitor_->startStateMonitor();

	planning_scene::PlanningScenePtr planning_scene_ = planning_scene_monitor_->getPlanningScene(); // Planning Scene extracted

	//---------------------------------------------------VIRTUAL JOINT TESTING--------------------------------

	std::cout << "Setting virtual_joint" << std::endl;
	robot_state::RobotState rover_state = planning_scene_->getCurrentStateNonConst();
	const std::vector<std::string> variable_names = rover_state.getVariableNames();

	for(int i=0 ; i<variable_names.size() ; i++)
	{
		std::cout << variable_names[i] << std::endl;
	}
	robot_state::RobotState current_state = planning_scene_->getCurrentStateNonConst();

	std::cout << "virtual_joint/x: " << current_state.getVariablePosition("virtual_joint/x") << std::endl;
	std::cout << "virtual_joint/y: " << current_state.getVariablePosition("virtual_joint/y") << std::endl;
	std::cout << "virtual_joint/theta: " << current_state.getVariablePosition("virtual_joint/theta") << std::endl;

	current_state.setVariablePosition("virtual_joint/x", 0.5);
	current_state.setVariablePosition("virtual_joint/y", 0.0);
	current_state.setVariablePosition("virtual_joint/theta", 3.14/2);
	planning_scene_->setCurrentState(current_state);

	robot_state::RobotState current_state_updated = planning_scene_->getCurrentStateNonConst();

	std::cout << "virtual_joint/x: " << current_state_updated.getVariablePosition("virtual_joint/x") << std::endl;
	std::cout << "virtual_joint/y: " << current_state_updated.getVariablePosition("virtual_joint/y") << std::endl;
	std::cout << "virtual_joint/theta: " << current_state_updated.getVariablePosition("virtual_joint/theta") << std::endl;


	std::cout << "virtual_joint set" << std::endl;

	//--------------------------------------------------------------------------------------------------------

	//--------------------------------------- Displaying Updated Robot States ---------------------------------

	ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("tutorial_robot_state", 1);
	ros::Rate loop_rate(1);

	for(float cnt=0 ; cnt<5 ; cnt++)
	{
		current_state_updated = planning_scene_->getCurrentStateNonConst();
		current_state_updated.setVariablePosition("virtual_joint/x", (cnt/2) );
		planning_scene_->setCurrentState(current_state_updated);
	
		moveit_msgs::DisplayRobotState robot_state_msg;
		robot_state::robotStateToRobotStateMsg(current_state_updated, robot_state_msg.state);

		robot_state_publisher.publish( robot_state_msg );

		ros::spinOnce();
		loop_rate.sleep();
	}


	//--------------------------------------------------------------------------------------------------------

	moveit_msgs::PlanningScene scene_object;

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.object.header.frame_id = "odom_combined";
	/* The id of the object */
	attached_object.object.id = "box";

	/* A default pose */
	geometry_msgs::Pose pose;
	pose.position.x = 1;
	pose.position.y = 0;
	pose.position.z = 0;
	pose.orientation.w = 1.0;

	/* Define a box to be attached */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 1;
	primitive.dimensions[1] = 1;
	primitive.dimensions[2] = 1;

	attached_object.object.primitives.push_back(primitive);
	attached_object.object.primitive_poses.push_back(pose);
	attached_object.object.operation = attached_object.object.ADD;

	scene_object.world.collision_objects.push_back(attached_object.object);

	//scene_object.robot_state = robot_state_msg; // Visualizing

	scene_object.is_diff = true;

	while (planning_scene_publisher_.getNumSubscribers() < 1)
	{
		ROS_WARN_ONCE("Waiting till planning_scene topic has subscribers...");
	}

	std::cout << "Subscribed!" << std::endl;

	planning_scene_publisher_.publish(scene_object);

	ros::spinOnce();
	sleep_t.sleep();

	//sleep_time.sleep();


	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	robot_state::RobotStatePtr rover_state_;
	rover_state_.reset(new robot_state::RobotState(planning_scene_->getCurrentStateNonConst()));
	//*rover_state_ = current_state_updated;

	std::cout << "rover_state_: " << std::endl;
	std::cout << "virtual_joint/x: " << rover_state_->getVariablePosition("virtual_joint/x") << std::endl;
	std::cout << "virtual_joint/y: " << rover_state_->getVariablePosition("virtual_joint/y") << std::endl;
	std::cout << "virtual_joint/theta: " << rover_state_->getVariablePosition("virtual_joint/theta") << std::endl;

	//collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();


	// Change robotstate by altering virtual joint values
	//std::cout << "frame_id: " << planning_scene_->getRobotModel()->getModelFrame() << std::endl;


	//---------------------------------------------------------TESTING MOVEIT VIRTUAL JOINT MOVE-----------------------
	// std::cout << "Setting virtual_joint" << std::endl;
	
	// const std::vector<std::string> variable_names = rover_state_->getVariableNames();

	// for(int i=0 ; i<variable_names.size() ; i++)
	// {
	// 	std::cout << variable_names[i] << std::endl;
	// }
	// robot_state::RobotState current_state = planning_scene_->getCurrentStateNonConst();

	// std::cout << "virtual_joint/x: " << current_state.getVariablePosition("virtual_joint/x") << std::endl;
	// std::cout << "virtual_joint/y: " << current_state.getVariablePosition("virtual_joint/y") << std::endl;
	// std::cout << "virtual_joint/theta: " << current_state.getVariablePosition("virtual_joint/theta") << std::endl;

	// current_state.setVariablePosition("virtual_joint/x", 1.0);
	// current_state.setVariablePosition("virtual_joint/y", 1.0);
	// current_state.setVariablePosition("virtual_joint/theta", 0);
	// planning_scene_->setCurrentState(current_state);

	// robot_state::RobotState current_state_updated = planning_scene_->getCurrentStateNonConst();

	// std::cout << "virtual_joint/x: " << current_state_updated.getVariablePosition("virtual_joint/x") << std::endl;
	// std::cout << "virtual_joint/y: " << current_state_updated.getVariablePosition("virtual_joint/y") << std::endl;
	// std::cout << "virtual_joint/theta: " << current_state_updated.getVariablePosition("virtual_joint/theta") << std::endl;


	// std::cout << "virtual_joint set" << std::endl;
	//------------------------------------------------------------------------------------------------------------------


	planning_scene_->checkCollision(collision_request, collision_result, *rover_state_);
	if (collision_result.collision)
	{
		std::cout << "Collision detected. Rejecting state" << std::endl;
	}
	else
	{
		std::cout << "No collision detected." << std::endl;
	}

	bool obj_exist = planning_scene_->getCollisionWorld()->getWorld()->hasObject("box");
	std::cout << "Object exists in collision world: " << obj_exist << std::endl;
	
	//boost::shared_ptr<int> p1(new int(1));



	while(1)
	{
		sleep_time.sleep();
	}

	ros::shutdown();
	return 0;

}