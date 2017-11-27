/*
 * Author: 	Abdul Moeed Zafar
 * Date:	28 Oct, 2017
 *
 * File:	rover_simulator.cpp
 */

 #include <ak_simulation/rover_simulator.h>
 #include <assert.h>

 #define RAD_TO_DEGREE M_PI/180

 namespace rover_simulation
 {

 	RoverSimulator::RoverSimulator() : node_handle_("~"), is_trajectory_set_(false)
	{
		trajectory_sub_ = node_handle_.subscribe("/planned_trajectory_topic", 100, &RoverSimulator::plannedTrajectoryCallBack, this);

		odom_base_tf_.header.frame_id = "odom_combined";
		odom_base_tf_.child_frame_id = "base_link";

		base_front_rw_tf_.header.frame_id = "base_link";
		base_front_rw_tf_.child_frame_id = "front_right_wheel";
		base_front_rw_tf_.transform.translation.x = 0.3;
		base_front_rw_tf_.transform.translation.y = -0.35;
		base_front_rw_tf_.transform.translation.z = 0.05;
		base_front_rw_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		base_front_lw_tf_.header.frame_id = "base_link";
		base_front_lw_tf_.child_frame_id = "front_left_wheel";
		base_front_lw_tf_.transform.translation.x = 0.3;
		base_front_lw_tf_.transform.translation.y = 0.35;
		base_front_lw_tf_.transform.translation.z = 0.05;
		base_front_lw_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		base_rear_rw_tf_.header.frame_id = "base_link";
		base_rear_rw_tf_.child_frame_id = "back_right_wheel";
		base_rear_rw_tf_.transform.translation.x = -0.3;
		base_rear_rw_tf_.transform.translation.y = -0.35;
		base_rear_rw_tf_.transform.translation.z = 0.05;
		base_rear_rw_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		base_rear_lw_tf_.header.frame_id = "base_link";
		base_rear_lw_tf_.child_frame_id = "back_left_wheel";
		base_rear_lw_tf_.transform.translation.x = -0.3;
		base_rear_lw_tf_.transform.translation.y = 0.35;
		base_rear_lw_tf_.transform.translation.z = 0.05;
		base_rear_lw_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		
		environment_obstacles_.publishObstacles();		
	}

	void RoverSimulator::plannedTrajectoryCallBack(const geometry_msgs::PoseArray& trajectory)
	{
		//std::cout << "INFO: Trajectory recieved!" << std::endl;
		trajectory_ = trajectory;
		std::cout << "Trajectory call back function called" << std::endl;
		is_trajectory_set_ = true;

		// ros::Duration(1).sleep();
		// publishTrajectory();
	}

	void RoverSimulator::setRoverPose(const geometry_msgs::Pose& rover_pose)
	{
		odom_base_tf_.transform.translation.x = rover_pose.position.x;
		odom_base_tf_.transform.translation.y = rover_pose.position.y;
		odom_base_tf_.transform.translation.z = ODOM_BASE_TF_Z;
		odom_base_tf_.transform.rotation = rover_pose.orientation;
	}

	void RoverSimulator::publishTrajectory()
	{

		assert(trajectory_.poses.size() > 0);

		ros::Rate loop_rate(30);

		//std::cout << "INFO: Publishing rover trajectory started" << std::endl;

		for(int i = 0 ; i < trajectory_.poses.size() ; i++)
		{
			ros::Time time_now = ros::Time::now();

			setRoverPose(trajectory_.poses[i]);
			odom_base_tf_.header.stamp = time_now;
			base_front_rw_tf_.header.stamp = time_now;
			base_front_lw_tf_.header.stamp = time_now;
			base_rear_rw_tf_.header.stamp = time_now;
			base_rear_lw_tf_.header.stamp = time_now;

			tf_broadcaster_.sendTransform(odom_base_tf_);
			tf_broadcaster_.sendTransform(base_front_rw_tf_);
			tf_broadcaster_.sendTransform(base_front_lw_tf_);
			tf_broadcaster_.sendTransform(base_rear_rw_tf_);
			tf_broadcaster_.sendTransform(base_rear_lw_tf_);

			loop_rate.sleep();
		}

		//std::cout << "INFO: Publishing rover trajectory ended" << std::endl;
	}

	void RoverSimulator::runSimulation()
	{
		// environment_obstacles_.publishObstacles();
		publishTrajectory();
	}


} //end namespace rover_simulation