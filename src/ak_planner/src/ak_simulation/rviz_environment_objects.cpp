/*
 * Author: 	Abdul Moeed Zafar
 * Date:	28 Oct, 2017
 *
 * File:	rviz_environment_objects.cpp
 */

#include <ak_simulation/rviz_environment_objects.h>


 namespace rover_simulation
 {

 	RvizEnvironmentObjects::RvizEnvironmentObjects() : node_handle_("~")
 	{
 		marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
 		marker_id_ = 0;

 		getObstacles();
 	}


 	visualization_msgs::Marker RvizEnvironmentObjects::createMarkerObject(float position_x, float position_y, float position_z, 
 			float orientation_x, float orientation_y, float orientation_z, float scale_x, float scale_y, float scale_z)
 	{
 		visualization_msgs::Marker marker;
		
		marker.header.frame_id = "odom_combined";
		marker.ns = "basic_shapes";
		marker.id = (marker_id_++);
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = position_x;
		marker.pose.position.y = position_y;
		marker.pose.position.z = position_z;
		marker.pose.orientation.x = orientation_x;
		marker.pose.orientation.y = orientation_y;
		marker.pose.orientation.z = orientation_z;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = scale_x;
		marker.scale.y = scale_y;
		marker.scale.z = scale_z;

		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0f;

		marker.lifetime = ros::Duration();

		return marker;
 	}


 	void RvizEnvironmentObjects::getObstacles()
 	{
 		// ENVIRONMENT_OBSTACLES.setupVisualizationEnvironment();

 		EnvironmentObstacles ENVIRONMENT_OBSTACLES;
 		ENVIRONMENT_OBSTACLES.setupVisualizationEnvironment();

 		for(int i = 0 ; i < ENVIRONMENT_OBSTACLES.obstacles.size() ; i++)
 		{
 			visualization_msgs::Marker marker;
 			marker = createMarkerObject(ENVIRONMENT_OBSTACLES.obstacles[i].at(0), ENVIRONMENT_OBSTACLES.obstacles[i].at(1), ENVIRONMENT_OBSTACLES.obstacles[i].at(2),
 				ENVIRONMENT_OBSTACLES.obstacles[i].at(3), ENVIRONMENT_OBSTACLES.obstacles[i].at(4), ENVIRONMENT_OBSTACLES.obstacles[i].at(5),
 				ENVIRONMENT_OBSTACLES.obstacles[i].at(6), ENVIRONMENT_OBSTACLES.obstacles[i].at(7), ENVIRONMENT_OBSTACLES.obstacles[i].at(8) );

 			object_vector_.push_back(marker);
 		}
 	}


 	bool RvizEnvironmentObjects::publishObstacles()
 	{
 		// assert(object_vector_.size() > 0);

 		while(marker_pub_.getNumSubscribers() < 1)
		{
			ROS_WARN_ONCE("Please create a subscriber to the marker.");
			sleep(1);
		}

		std::cout << "object_vector_.size() : " << object_vector_.size() << std::endl;

		for(int i = 0 ; i < object_vector_.size() ; i++)
 		{
 			marker_pub_.publish(object_vector_[i]);
 		}
		
 	}



 } //end namespace rover_simulation