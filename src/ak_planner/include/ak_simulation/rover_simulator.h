/*
 * Author: 	Abdul Moeed Zafar
 * Date:	28 Oct, 2017
 *
 * File:	rover_simulator.h
 */

#ifndef _ROVER_SIMULATOR_H_
#define _ROVER_SIMULATOR_H_


#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>

#include <ak_simulation/shared_variables.h>
#include <ak_simulation/rviz_environment_objects.h>


namespace rover_simulation
{
	class RoverSimulator
	{

	private:

		ros::NodeHandle node_handle_;
		tf::TransformBroadcaster tf_broadcaster_;

		geometry_msgs::TransformStamped odom_base_tf_;
		geometry_msgs::TransformStamped base_front_rw_tf_;
		geometry_msgs::TransformStamped base_front_lw_tf_;
		geometry_msgs::TransformStamped base_rear_rw_tf_;
		geometry_msgs::TransformStamped base_rear_lw_tf_;

		geometry_msgs::PoseArray trajectory_;
		ros::Subscriber trajectory_sub_;

		RvizEnvironmentObjects environment_obstacles_;

		bool is_trajectory_set_;
		

		void setRoverPose(const geometry_msgs::Pose& rover_pose);


	public:

		RoverSimulator();
		void plannedTrajectoryCallBack(const geometry_msgs::PoseArray& trajectory);
		void publishTrajectory();

		void runSimulation();

		inline bool isTrajectoryRecieved() const;

	};

	inline bool RoverSimulator::isTrajectoryRecieved() const
	{
		return is_trajectory_set_;
	}

} //end namespace rover_simulation

#endif