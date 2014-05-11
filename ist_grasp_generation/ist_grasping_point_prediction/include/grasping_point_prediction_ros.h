#ifndef GRASPING_POINT_PREDICTION_ROS_H
#define GRASPING_POINT_PREDICTION_ROS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ist_grasp_generation_msgs/GraspingPointPrediction.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <grasping_point_prediction.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>


	
class GraspingPointPredictionRos
{
	private:
		// The node handler
  		ros::NodeHandle _n;

		// The private node handler
  		ros::NodeHandle _n_priv;

		// Grasping point prediction service
		ros::ServiceServer grasping_point_prediction_service;

		// A tf transform listener
		tf::TransformListener listener;


		tf::StampedTransform table_transform_stamped;

		tf::StampedTransform object_transform_stamped;

		tf::StampedTransform handle_transform_stamped;
	
		std::string world_frame_id;


		GraspingPointPrediction grasping_point_prediction;

		/////////////
		// METHODS //
		/////////////

		// Service for point cloud refinement
		bool graspingPointPredictionServiceCallback(ist_grasp_generation_msgs::GraspingPointPrediction::Request  &req, ist_grasp_generation_msgs::GraspingPointPrediction::Response &res);

	public:

		GraspingPointPredictionRos(ros::NodeHandle & n);

};

#endif //#ifndef GRASPING_POINT_PREDICTION_ROS_H
