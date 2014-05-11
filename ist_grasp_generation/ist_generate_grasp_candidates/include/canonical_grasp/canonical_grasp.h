/*
 * canonical_grasp.h
 *
 *  Created on: Dec 15, 2012
 *      Author: rui
 */

#ifndef CANONICALGRASP_H_
#define CANONICALGRASP_H_

#define EIGEN_USE_NEW_STDVECTOR
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/StdVector>

#include "grasp_type/grasp_type.h"
#include "hand_posture/hand_posture.h"
#include <map>

class CanonicalGrasp : public GraspType
{
	protected:
		// Static definitions (grip coordinate axis)
		static const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> x_g;
		static const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> y_g;
		static const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> z_g;

	public:
  		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		// List containing all possible canonical grasps
//		static const std::vector<CanonicalGrasp,Eigen::aligned_allocator<CanonicalGrasp> > canonical_grasps;
  		static std::map<unsigned int, boost::shared_ptr<CanonicalGrasp> > canonical_grasps;

		// Grasp number (discrete) (1 to 33)
		unsigned int number;

		// Name of the grasp type
		std::string grasp_name;

		// Orientation type with respect to grip
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> orientation;

		// Canonical continuous offsets, loaded from config file
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pose_offset;

		// Grasp pre-pose hand posture
		HandPosture open_hand_posture;

		// Grasp pose hand posture
		HandPosture closed_hand_posture;

		// Pre grasp distance threshold
		double pre_grasp_distance;



		// Perturbations
		double perturbation_angle_step;
		int n_perturbs;
		Eigen::Vector3d perturbation_rotation_axis;

		// Constructors
		CanonicalGrasp()
		{};

		// Constructors
		CanonicalGrasp(int _number) : number(_number)
		{};

//		CanonicalGrasp(int _type, int _number);

		CanonicalGrasp(int _type, unsigned int _id, std::string _grasp_name, Eigen::AngleAxisd & _orientation, Eigen::Transform<double, 3, Eigen::Affine> & _pose_offset, HandPosture & _open_hand_posture, HandPosture & _closed_hand_posture, double _pre_grasp_distance, const double _perturbation_angle_step, const unsigned int _n_perturbs, const Eigen::Vector3d & _perturbation_rotation_axis) :
			GraspType(_type), number(_id),
			grasp_name(_grasp_name),
			orientation(_orientation),
			pose_offset(_pose_offset),
			open_hand_posture(_open_hand_posture),
			closed_hand_posture(_closed_hand_posture),
			pre_grasp_distance(_pre_grasp_distance),
			perturbation_angle_step(_perturbation_angle_step),
			n_perturbs(_n_perturbs),
			perturbation_rotation_axis(_perturbation_rotation_axis)
		{
//			std::cout << "new canonical grasp: " << std::endl;
//			std::cout << "name:" << grasp_name << std::endl;
//			std::cout << "id:" << number << std::endl;
//
//			std::cout << "canonical_grasp_orientation:" << std::endl << orientation.matrix() << std::endl;
//			std::cout << "orientation_offset_:" <<  std::endl << offset_rotation.matrix() << std::endl;
//			std::cout << "translation_offset_:" <<  std::endl << offset_translation.matrix() << std::endl;
		};

		static void printInfo()
		{
//			for(int32_t i=0; i<canonical_grasps.size(); ++i)
//			{
//				std::cout << "name:" << canonical_grasps[i].grasp_name << std::endl;
//				std::cout << "id:" << canonical_grasps[i].number << std::endl;
//
//				std::cout << "canonical_grasp_orientation:" << std::endl << canonical_grasps[i].orientation.matrix() << std::endl;
//				std::cout << "orientation_offset_:" <<  std::endl << canonical_grasps[i].offset_rotation.matrix() << std::endl;
//				std::cout << "translation_offset_:" <<  std::endl << canonical_grasps[i].offset_translation.matrix() << std::endl;
//			}
			//std::cout << "grasp_position_:" << grasp_position_.matrix() << std::endl;
			//std::cout << "pre_grasp_position_:" << pre_grasp_position_.matrix() << std::endl;
		}

};

#endif /* CANONICALGRASP_H_ */
