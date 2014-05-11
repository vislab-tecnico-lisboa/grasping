/*
 * ReachabilityMap.h
 *
 *  Created on: Oct 23, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef REACHABILITYMAP_H_
#define REACHABILITYMAP_H_

#include <Eigen/Eigen>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

#include <canonical_grip/canonical_grip.h>

#include <ist_grasp_generation_msgs/InverseKinematics.h>
#include <ist_grasp_generation_msgs/InverseKinematicsBatch.h>


#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <visualization_msgs/MarkerArray.h>
#define PI 3.14159265
#define RAD_TO_DEG 180/PI




class ReachabilityMap
{
	private:
		double x_length_;
		double y_length_;
		double z_length_;

		int x_bins_;
		int y_bins_;
		int z_bins_;
		int yaw_bins_;
		int pitch_bins_;
		int roll_bins_;

		double x_step_;
		double y_step_;
		double z_step_;
		double yaw_step_;
		double pitch_step_;
		double roll_step_;


		int x_perturbs_;
		int y_perturbs_;
		int z_perturbs_;
		int yaw_perturbs_;
		int pitch_perturbs_;
		int roll_perturbs_;

		double x_perturb_step_;
		double y_perturb_step_;
		double z_perturb_step_;
		double yaw_perturb_step_;
		double pitch_perturb_step_;
		double roll_perturb_step_;

		double total_time_;
		unsigned int total_iterations_;

		Eigen::Vector3d offset_;

		void train();
		void batchTrain();

		void quaternionToEulerTest(const Eigen::Quaternion<double> & q, double & roll, double & pitch, double & yaw);

		void quaternionToEuler(const Eigen::Quaternion<double> & q, double & roll, double & pitch, double & yaw);
		void eulerToQuaternion(Eigen::Quaternion<double> & q, const double & roll, const double & pitch, const double & yaw);
		void fixedAnglesToRotationMatrix(const double & roll, const double & pitch, const double & yaw, Eigen::Matrix3d & perturb_rotation);

		///////////////
		// ROS STUFF //
		///////////////

		ros::NodeHandle n_, n_priv_;

		ist_grasp_generation_msgs::InverseKinematics InverseKinematicsSrv_;
		ist_grasp_generation_msgs::InverseKinematicsBatch InverseKinematicsBatchSrv_;


		ros::ServiceClient inverse_kinematics_client_;
		ros::ServiceClient inverse_kinematics_batch_client_;

		ros::Publisher reachability_grid_pub_;

		std::string world_frame_id_;
		std::string arm_frame_id_;

		tf::StampedTransform world_to_arm_transform_; // arm pose in world frame
		geometry_msgs::Pose world_to_arm_msg_;

		void displayGrid();


		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			ar & x_length_;
			ar & y_length_;
			ar & z_length_;

			ar & x_bins_;
			ar & y_bins_;
			ar & z_bins_;
			ar & yaw_bins_;
			ar & pitch_bins_;
			ar & roll_bins_;

			ar & x_step_;
			ar & y_step_;
			ar & z_step_;
			ar & yaw_step_;
			ar & pitch_step_;
			ar & roll_step_;

			ar & x_perturbs_;
			ar & y_perturbs_;
			ar & z_perturbs_;
			ar & yaw_perturbs_;
			ar & pitch_perturbs_;
			ar & roll_perturbs_;

			ar & x_perturb_step_;
			ar & y_perturb_step_;
			ar & z_perturb_step_;
			ar & yaw_perturb_step_;
			ar & pitch_perturb_step_;
			ar & roll_perturb_step_;

			ar & reachability_successes;
			ar & reachability_failures;
			ar & reachability_map;

			ar & total_iterations_;
			ar & total_time_;
		}

		bool saveData(std::string filename)
		{
		    // create and open a character archive for output
		   	std::ofstream ofs(filename.c_str());
		   	if(ofs.fail())
		   		return false;
		    // save data to archive
		    {
		   		boost::archive::binary_oarchive oa(ofs);
		   		// write class instance to archive

		   		oa << *this;
		   		// archive and stream closed when destructors are called
		    }
		    return true;
		}

		bool loadData(std::string filename)
		{
			// open a character archive for input
		    std::ifstream ifs(filename.c_str());
		    if(ifs.fail())
		    	return false;

		    // load data from archive
		    {
				// write class instance to archive
		    	boost::archive::binary_iarchive ia(ifs);

				ia >> *this;
		    }
		    return true;
		}

	public:
		std::vector <std::vector <std::vector <std::vector <unsigned int > > > > reachability_successes;
		std::vector <std::vector <std::vector <std::vector <unsigned int > > > > reachability_failures;
		std::vector <std::vector <std::vector <std::vector <double > > > > reachability_map;


		ReachabilityMap(){};
		/*ReachabilityMap(
				ros::NodeHandle & n,
				std::string & world_frame_id,
				std::string & arm_frame_id,
				std::string & file_name,
				const double & x_length,
				const double & y_length,
				const double & z_length,
				const unsigned int & x_bins,
				const unsigned int & y_bins,
				const unsigned int & z_bins,
				const unsigned int & x_perturbs,
				const unsigned int & y_perturbs,
				const unsigned int & z_perturbs,
				const unsigned int & roll_perturbs,
				const unsigned int & pitch_perturbs,
				const unsigned int & yaw_perturbs
				);
		*/
		ReachabilityMap(ros::NodeHandle & n);

		void getMapIndex(const double & x, const double & y, const double & z, const double & roll, const double & pitch, const double & yaw);

		//int poseToIndex(Eigen::Transform<double,3,Eigen::Affine> & pose);
		int poseToIndex(Eigen::Transform<double,3,Eigen::Affine> & pose, int & orientation_index);

		virtual ~ReachabilityMap();
};

#endif /* REACHABILITYMAP_H_ */
