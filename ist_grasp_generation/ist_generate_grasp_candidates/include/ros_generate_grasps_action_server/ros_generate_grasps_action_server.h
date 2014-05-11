/*
 * ros_generate_grasps_action_server.h
 *
 *  Created on: Jan 8, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef ROS_GENERATE_GRASPS_ACTION_SERVER_H_
#define ROS_GENERATE_GRASPS_ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ist_grasp_generation_msgs/GetGeneratedGraspsAction.h>
#include <ist_msgs/GripList.h>
#include <eigen_conversions/eigen_msg.h>
#include "generate_grasps/generate_grasps.h"
//#include "ist_babbling/Process_List.h"
//#include "ist_babbling/New_Process.h"
//#include "ist_babbling/Learn.h"
//#include "ist_babbling/ProcessInfo.h"
//#include "ist_babbling/Find_Best.h"
//#include "ist_babbling/Plan.h"
#include "ist_grasp_generation_msgs/NewExperiment.h"


#include <boost/bimap.hpp>
#include "definitions.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>

class RosGenerateGraspsActionServer
{
	protected:
		// The node handler
		ros::NodeHandle nh_;

		// The private node handler
		ros::NodeHandle n_priv;

		actionlib::SimpleActionServer<ist_grasp_generation_msgs::GetGeneratedGraspsAction> as_;
		std::string action_name_;

		// Create messages that are used to published feedback/result
		ist_grasp_generation_msgs::GetGeneratedGraspsFeedback feedback_;
		ist_grasp_generation_msgs::GetGeneratedGraspsResult result_;

		// Generate grasps new experiment list service
		ros::ServiceServer generate_grasps_new_experiment_service_;

		// Babbling Process_List service client
//		ist_babbling::Process_List srv_BabblingProcessList;
//		ros::ServiceClient babbling_processing_list_client_;
//
//		// Babbling New_Process service client
//		ist_babbling::New_Process srv_BabblingNewProcess;
//		ros::ServiceClient babbling_new_process_client_;
//
//		// Babbling Find_Best service client
//		ist_babbling::Find_Best srv_BabblingFindBest;
//		ros::ServiceClient babbling_find_best_client_;
//
//		// Babbling Plan service client
//		ist_babbling::Plan srv_BabblingPlan;
//		ros::ServiceClient babbling_plan_client_;
//
//		// Babbling Learn service client
//		ist_babbling::Learn srv_BabblingLearn;
//		ros::ServiceClient babbling_learn_client_;

		// Prediction mode
		std::string predict_mode;

		// Learn mode activated
		bool learn;

		// Offset from hand base to palm center
		double palm_center_offset_x;
		double palm_center_offset_y;
		double palm_center_offset_z;

		// Maps (handle definitions)
		std::map<std::pair<int,int>,int> size_shape_object_map;
		std::map<int,int> msgs_size_handle_size_map;
		// Existing babbling map
		std::map<std::string, bool > grasp_to_process_name;
		boost::bimap<unsigned int, std::string > graspability_state_gaussian_process_bimap;

	public:
		// Constructors
		RosGenerateGraspsActionServer(std::string name);

		virtual ~RosGenerateGraspsActionServer();

		void executeCB(const ist_grasp_generation_msgs::GetGeneratedGraspsGoalConstPtr &goal);

		// New experiment service callback
		bool updateGaussianProcessesCallback(ist_grasp_generation_msgs::NewExperiment::Request & req, ist_grasp_generation_msgs::NewExperiment::Response & res);

		// Method used to fill ist_msgs/GripPose.msg (using grasps)
		ist_msgs::GripPose fillGripPoseMessage(const boost::shared_ptr<Grasp> grasp, const std::string & frame_id, const ist_msgs::ObjectPart & object_part);

		// Method used to fill ist_msgs/GripPose.msg (using grasp perturbations)
		//ist_msgs::GripPose fillGripPoseMessage(const boost::shared_ptr<GraspPerturbation> grasp_perturbation, const std::string & frame_id, const ist_msgs::ObjectPart & object_part);

		// Method used to fill ist_msgs/HandState.msg (using grasps)
		ist_msgs::HandState fillHandStateMessage(const boost::shared_ptr<Grasp> grasp, std::string frame_id);

		// Method used to fill ist_msgs/HandState.msg (using grasp perturbations)
//		ist_msgs::HandState fillHandStateMessage(const boost::shared_ptr<GraspPerturbation> grasp_perturbation, std::string frame_id);

		boost::shared_ptr<GenerateGrasps> graspability_states;

};
#endif /* ROS_GENERATE_GRASPS_ACTION_SERVER_H_ */
