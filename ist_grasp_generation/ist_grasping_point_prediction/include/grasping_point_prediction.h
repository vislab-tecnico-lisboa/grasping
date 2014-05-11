#ifndef GRASPING_POINT_PREDICTION_H
#define GRASPING_POINT_PREDICTION_H

#include <libsvm/svm.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/filters/crop_box.h>
#include <vector>
#include <pcl/filters/passthrough.h>
class GraspingPointPrediction
{
	private:

		Eigen::Vector4f _maxPoint, _minPoint;
		double _mini_box_x_dim;
		double _mini_box_y_dim;
		double _mini_box_z_dim;
	
		double _mini_box_x_bins;
		double _mini_box_y_bins;

		double _mini_box_pixel_x_step;
		double _mini_box_pixel_y_step;
		double _mini_box_pixel_x_step_inverse;
		double _mini_box_pixel_y_step_inverse;

		double _x_offset, _y_offset;

		svm_model * _model;

		std::vector<std::vector < double > > _image_grid;


		std::vector <double> computeFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr & data);

		std::vector<double> predictGraspingPoint(std::vector<std::vector<double> > & feature_groups);
	public:
		GraspingPointPrediction();
		GraspingPointPrediction(std::string & file_name, double mini_box_x_dim, double mini_box_y_dim, double  mini_box_z_dim, unsigned int mini_box_x_bins, unsigned int mini_box_y_bins);
		
		std::vector<double> computeGraspingProbability(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> & data);

};

#endif //#ifndef GRASPING_POINT_PREDICTION_H
