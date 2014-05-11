#include "grasping_point_prediction_ros.h"
#include <pcl/visualization/cloud_viewer.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	const std::string points="points";
	const std::string normals="normals";
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);

 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);

  	viewer->addPointCloud<pcl::PointXYZI> (cloud, single_color, points,0);
  	//viewer->addPointCloudNormals<pcl::PointXYZI>(cloud,1,0.01,"normals", 0);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, points);
  	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals"); 
  	viewer->addCoordinateSystem (0.1);
  	viewer->initCameraParameters ();
  	return (viewer);
}

GraspingPointPredictionRos::GraspingPointPredictionRos(ros::NodeHandle & n) : _n(n), _n_priv("~")
{
	////////////////
	// Parameters //
	////////////////

	std::string model_file_name;

	double mini_box_x_dim;
	double mini_box_y_dim;
	double mini_box_z_dim;

	int mini_box_x_bins;
	int mini_box_y_bins;

	_n_priv.param<std::string>("model_file_name", model_file_name, "/home/vislab/svnRepositories/ros/grasping/grasping_point_prediction_stack/grasping_point_prediction/data/modelLearnt.txt");
	ROS_INFO("model file name: %s", model_file_name.c_str());

	//std::cout << model_file << std::endl;

	_n_priv.param<double>("mini_box_x_dim", mini_box_x_dim, 0.15);
	ROS_INFO("mini box x dim: %f", mini_box_x_dim);

	_n_priv.param<double>("mini_box_y_dim", mini_box_y_dim, 0.07);
	ROS_INFO("mini box y dim: %f", mini_box_y_dim);

	_n_priv.param<double>("mini_box_z_dim", mini_box_z_dim, 0.15);
	ROS_INFO("mini box z dim: %f", mini_box_z_dim);

	_n_priv.param<int>("mini_box_x_bins", mini_box_x_bins, 21);
	ROS_INFO("mini box pixel x step: %d", mini_box_x_bins);

	_n_priv.param<int>("mini_box_y_bins", mini_box_y_bins, 10);
	ROS_INFO("mini box pixel y step: %d", mini_box_y_bins);

	grasping_point_prediction=GraspingPointPrediction(model_file_name, mini_box_x_dim, mini_box_y_dim, mini_box_z_dim, (unsigned int)mini_box_x_bins, (unsigned int)mini_box_y_bins);

	//////////////
	// Services //
	//////////////

	// Point cloud refinement service
  	grasping_point_prediction_service = n.advertiseService("grasping_point_prediction", &GraspingPointPredictionRos::graspingPointPredictionServiceCallback,this);
}


pcl::PointCloud<pcl::PointXYZI>::Ptr preProcessData(geometry_msgs::PoseStamped & grip_pose, geometry_msgs::PoseStamped object_pose, pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, int object_part)
{
	Eigen::Vector3d x_o(1.0, 0.0, 0.0);
	Eigen::Vector3d y_o(0.0, 0.0, -1.0);
	Eigen::Vector3d z_o(0.0, 1.0, 0.0);

	Eigen::Matrix<double, 3, 3> rotation_matrix;
		rotation_matrix <<  x_o,  y_o,  z_o;

	Eigen::Transform<double, 3, Eigen::Affine> orientation=Eigen::Transform<double, 3, 3>(rotation_matrix);
	
	std::cout << "object part:" << object_part << std::endl;


	///////////////////////////////////////
	// Convert point cloud to hand frame //
	///////////////////////////////////////

	Eigen::Affine3d object_to_hand_transform;// Hand pose in object frame
	tf::poseMsgToEigen(grip_pose.pose, object_to_hand_transform);


	Eigen::Affine3d object_to_box_transform;// Object pose in world frame
	tf::poseMsgToEigen(object_pose.pose, object_to_box_transform);


	pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);

	Eigen::Affine3d combined_transformation =  orientation *object_to_hand_transform.inverse();// * orientation;

	pcl::transformPointCloud(*point_cloud, *point_cloud_transformed, (Eigen::Affine3f) combined_transformation);

	pcl::PointCloud<pcl::PointXYZI>::Ptr part_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	for(pcl::PointCloud<pcl::PointXYZI>::iterator p=point_cloud_transformed->begin(); p<point_cloud_transformed->end(); ++p)
	{
		//if(p->intensity==object_part)
		//{
			part_point_cloud->push_back(*p);
		//}
	}

	return part_point_cloud;
}

bool GraspingPointPredictionRos::graspingPointPredictionServiceCallback(ist_grasp_generation_msgs::GraspingPointPrediction::Request  &req, ist_grasp_generation_msgs::GraspingPointPrediction::Response &res)
{
	ROS_INFO("Grasping point prediction service...");

	sensor_msgs::PointCloud2 ros_point_cloud2;
	
	sensor_msgs::convertPointCloudToPointCloud2(req.object.state.graspable_object.cluster, ros_point_cloud2); // convert to sensor_msgs/PointCloud2
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(ros_point_cloud2, *pcl_point_cloud); // convert to sensor_msgs/PointCloud2 to pcl

		
	//std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > data_samples;

	std::cout << "NUMBER OF GRASPS: " << req.grips.grip_states.size() << std::endl;
	res.evaluated_grips=req.grips;
	// For each grasp...
	for(int i=0; i< req.grips.grip_states.size(); ++i)
	{
		std::cout << "grasp index:" << i << std::endl;
		//if(req.grips.grip_states[i].current_trajectory_status==1) 
		{

			// One data sample at a time
			std::vector < pcl::PointCloud<pcl::PointXYZI>::Ptr > data_samples;


			geometry_msgs::PoseStamped grasp_pose_msg;

			// Check if grip has perturbation
			//if(req.grips.grip_states[i].grip_pose.perturb)
			//{
			//	std::cout << "perturbation grasp" << std::endl;
				// BEM FEITO SERIA SE A PERTURBAÇÃO FOSSE RELATIVE AO GRIP, MAS ESTA E UM GRASP NOVO
			grasp_pose_msg=req.grips.grip_states[i].hand_state.grasp_pose.pose;
			//	grasp_pose_msg=req.grips.grip_states[i].grip_pose.perturb_pose;
			//}
			//else
			//{
			//	std::cout << "original grasp" << std::endl;
			//	grasp_pose_msg=req.grips.grip_states[i].hand_state.grasp_pose.pose;
			//}
			
			data_samples.push_back(preProcessData(grasp_pose_msg, req.object.state.graspable_object.potential_models[0].pose, pcl_point_cloud, req.grips.grip_states[i].grip_pose.part.id));
			
			// Compute grasp probability
			std::vector<double> grasping_point_probabilities=grasping_point_prediction.computeGraspingProbability(data_samples);
			std::cout << " probability before: " << grasping_point_probabilities[0] << std::endl;
			double prob=1.0 / ( 1.0+exp( (2.0*grasping_point_probabilities[0])+0.5 ) );
			std::cout << " probability after: " << prob << std::endl;
			res.evaluated_grips.grip_states[i].success_probability=prob;
		}
		//else
		//	res.evaluated_grips.grip_states[i].success_probability=0.0;
			
	}

	/*res.scores=grasping_point_probabilities;
	for(int i=0; i< grasping_point_probabilities.size(); ++i)
	{
		std::cout << grasping_point_probabilities[i] << std::endl;
		//std::cout << res.scores << " ";
	}
	std::cout <<  std::endl;*/
	ROS_INFO("Done");

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grasping_point_prediction");
	ros::NodeHandle n;

	GraspingPointPredictionRos graspingPointPrediction(n);

	ros::spin();
}
