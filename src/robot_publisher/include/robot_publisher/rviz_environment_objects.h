/*
 * Author: 	Abdul Moeed Zafar
 * Date:	28 Oct, 2017
 *
 * File:	rviz_environment_objects.h
 */

#ifndef _RVIZ_ENVIRONMENT_OBJECTS_H_
#define _RVIZ_ENVIRONMENT_OBJECTS_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <robot_publisher/shared_variables.h>

 namespace rover_simulation
 {

 	class RvizEnvironmentObjects
 	{

 	private:

 		ros::NodeHandle node_handle_;

 		ros::Publisher marker_pub_;
 		std::vector<visualization_msgs::Marker> object_vector_;

 		int marker_id_;


 		visualization_msgs::Marker createMarkerObject(float position_x, float position_y, float position_z, 
 			float orientation_x, float orientation_y, float orientation_z, float scale_x, float scale_y, float scale_z);

 	public:

 		RvizEnvironmentObjects();
 		void getObstacles();
 		bool publishObstacles();

 		std::vector<visualization_msgs::Marker> getObjectVector() {	return object_vector_; }


 	};

 } //end namespace rover_simulation



#endif