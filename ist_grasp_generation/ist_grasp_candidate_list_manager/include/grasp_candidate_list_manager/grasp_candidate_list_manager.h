/*
 * grasp_candidate_list_manager.h
 *
 *  Created on: Dec 15, 2012
 *      Author: Rui P. Figueiredo
 */

#ifndef GRASPCANDIDATELISTMANAGER_H_
#define GRASPCANDIDATELISTMANAGER_H_
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/assign.hpp> // for 'map_list_of()'
#include <boost/assert.hpp>
#include <ist_grasp_generation_msgs/GetEvaluatedGrasps.h>
#include <ist_grasp_generation_msgs/NewExperiment.h>
#include "human_knowledge_parser/csv_parser.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <database_access_msgs/GetGraspabilityMap.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>

#include "ReachabilityMap.h"

#define GRASP_SPACE_SIZE 33
#define GRIP_SPACE_SIZE 24
#define PART_SPACE_SIZE 0 //26
#define DISTANCE_SPACE_SIZE 3
#define SHAPE_SPACE_SIZE 4
#define SYMMETRY_SPACE_SIZE 0 //12

#define ROBOT true
#define HUMAN false

#define SIZE_SPACE_SIZE 10
#define OBJECT_SPACE_SIZE 25

#include <time.h>


class GraspCandidateListManager
{
	private:
		// The node handler
		ros::NodeHandle n;

		// The private node handler
		ros::NodeHandle n_priv;

//		database_access_msgs::GetGraspabilityMap srv_GetGraspabilityMap;
		ros::ServiceClient get_database_graspability_map_client_;

		// Grasp candidate list manager service
		ros::ServiceServer grasp_candidate_list_manager_service;

		// Grasp candidate list new experiment service
		ros::ServiceServer grasp_candidate_list_manager_new_experiment_service;

		// Gripper size
		double gripper_size;

		// World frame name
		std::string world_frame_id;

		// Table frame name
		std::string table_frame_id;

		// Set coordinate frame broadcaster service
		ros::ServiceServer display_marker_service;

		// Grasps markers publisher
		ros::Publisher marker_pub;

		// Grasps marker array
		visualization_msgs::MarkerArray marker_array;

		bool use_reachability_map;

		static std::vector<std::vector<bool> > grip_equivalence; // map of size type and shape type (we don't consider symmetry)

		//////////////////////////////////////////////////////
		// Propagation rules according to object symmetries //
		//////////////////////////////////////////////////////

		static std::vector<std::vector<double> > symmetry_1ALL_grip_equivalence_rules; // (BRICK, ERASER)
		static std::vector<std::vector<double> > symmetry_2ALL_grip_equivalence_rules; // (SPHERE, CUBE) ALL GRASPS IDENTICAL
		static std::vector<std::vector<double> > symmetry_3Z_grip_equivalence_rules;   // (CHALK STICK, ROLLING PIN)

		//////////////////////////////////////////////
		// Propagation rules according to distances //
		//////////////////////////////////////////////

		std::vector<std::vector<double> > distance_kernel;
		std::vector<std::vector<double> > grasp_similarity_kernel;

		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > temp_successes;
		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > temp_failures;
		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > graspability_human_successes;
		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > graspability_robot_successes;
		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > graspability_human_failures;
		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > graspability_robot_failures;
		std::vector <std::vector <std::vector <std::vector <std::vector <std::vector <double > > > > > > graspability_map;

		std::map<std::pair<int,int>,int> size_shape_object_map;
		std::map<int,int> object_shape_map;
		std::map<int,int> msgs_size_handle_size_map;


		// Reachability map
		ReachabilityMap reachability_map;

		// Method used to publish grasps markers
		void fillGraspsMarkers(ist_msgs::GripList & grasps, ist_msgs::Object & object);

		void incorporateKnowledge(
				unsigned int grasp,
				unsigned int grip,
				unsigned int part,
				unsigned int distance,
				unsigned int object,
				unsigned int symmetry,
				double trials,
				double successes,
				bool is_robot,
				bool success=true);

		void graspabilityMapToImage();
		void graspabilityMapSingleObjectToImage(const unsigned int object_index);

		ist_msgs::GripList discardGrasps(ist_msgs::GripList & grasps, ist_msgs::Object & object);

		ist_msgs::GripList handObjectRules(ist_msgs::GripList & grasps, ist_msgs::Object & object);

		ist_msgs::GripList handWorldRules(ist_msgs::GripList & grasps, ist_msgs::Object & object);

		image_transport::ImageTransport it_;
//        sensor_msgs::cv bridge_;	// Cv bridge to send graspability_map
		image_transport::Publisher graspability_map_image_pub_;
		image_transport::Publisher graspability_object_map_image_pub_;

	public:

		// Constructors
		GraspCandidateListManager(ros::NodeHandle n_);

		// Destructor
		virtual ~GraspCandidateListManager();

		// Service call back
		bool serviceCallback(ist_grasp_generation_msgs::GetEvaluatedGrasps::Request  &req, ist_grasp_generation_msgs::GetEvaluatedGrasps::Response &res);

		bool newExperimentServiceCallback(ist_grasp_generation_msgs::NewExperiment::Request &req, ist_grasp_generation_msgs::NewExperiment::Response &res);
};

#endif /* GRASPCANDIDATELISTMANAGER_H_ */
