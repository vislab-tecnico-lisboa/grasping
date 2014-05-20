/*
 * grasp.h
 *
 *  Created on: Dec 15, 2012
 *      Author: Rui P. Figueiredo
 */

#ifndef GRASP_H_
#define GRASP_H_

#include "canonical_grasp/canonical_grasp.h"
#include "grip/grip.h"
#include "reach_type/reach_type.h"
#include "reach/reach.h"
#include "object_type/object_type.h"
#include "normalizing_functions.h"
#include "definitions.h"
#include "canonical_object_part/canonical_object_part.h"
#include "grasp_perturbation/grasp_perturbation.h"

class Grasp
{
	private:
		// Grasp pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> grasp_object_center_pose;

		// Grasp pre-pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> grasp_object_center_pre_pose;
		void normalizedEulerToEigen(const double _normalized_yaw,
									const double _normalized_pitch,
									const double _normalized_roll,
									Eigen::Transform<double, 3, Eigen::Affine,
									Eigen::DontAlign> & _rotation);

		void eigenToEuler(  const Eigen::Transform<double, 3,
							Eigen::Affine> & transform,
							double & _yaw,
							double & _pitch,
							double & _roll);

		void getNormalizedYawPitchRoll(const Eigen::Transform<double, 3, Eigen::Affine> & _rotation,
									   double & _normalized_yaw,
									   double & _normalized_pitch,
									   double & _normalized_roll);

		void getNormalizedTranslation(const Eigen::Transform<double, 3, Eigen::Affine> & _translation,
									  double & _normalized_x,
									  double & _normalized_y, double & _normalized_z);

		void denormalizedTranslation(const double _normalized_x,
									 const double _normalized_y,
									 const double _normalized_z,
									 Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> & _translation);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Grasp id - object type (1-25) + grasp type (1-33) + grip type (1-24) + reach type (1-3)
		unsigned int id;

		// Canonical grasp associated with this grasp
		boost::shared_ptr<CanonicalGrasp> canonical_grasp;

		// Reach type associated with this grasp
		Reach reach;

		// Grip associated with this grasp
		Grip grip;

		// Perturbation associated with this grasp
		GraspPerturbation grasp_perturbation;

        // Hand canonical pose
        Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> hand_canonical_pose;

		// Object type associated with this grasp
		boost::shared_ptr<ObjectType> object_type;

		// Canonical object part associated with this grasp
		boost::shared_ptr<ObjectPart> object_part;

		// Grasp pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pose;

		// Grasp pre-pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pre_pose;

		// Continuous offsets (associated with gaussian processes)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> offset_rotation;
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> offset_translation;


		void getNormalizedParameters(double & _normalized_yaw,
									 double & _normalized_pitch,
									 double & _normalized_roll,
									 double & _normalized_x,
									 double & _normalized_y,
									 double & _normalized_z,
									 double & _normalized_reach_distance,
									 double & _normalized_object_dimension_x,
									 double & _normalized_object_dimension_y,
									 double & _normalized_object_dimension_z);

		void setParameters(const double _normalized_yaw,
						   const double _normalized_pitch,
						   const double _normalized_roll,
						   const double _normalized_x,
						   const double _normalized_y,
						   const double _normalized_z,
						   const double _normalized_reach_distance,
						   const double _normalized_object_dimension_x,
						   const double _normalized_object_dimension_y,
						   const double _normalized_object_dimension_z);

		// Constructors
		Grasp();

		void updateGrasp(const Eigen::Vector3d & _object_part_dimensions, const Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> & _object_part_pose);

//		Grasp(const boost::shared_ptr<CanonicalGrasp> & _canonical_grasp, const Reach & _reach, const Grip & _grip, const boost::shared_ptr<ObjectType> & _object_type, const boost::shared_ptr<ObjectPart> & _object_part);

		Grasp(const boost::shared_ptr<CanonicalGrasp> & _canonical_grasp, const Reach & _reach, const Grip & _grip, const boost::shared_ptr<ObjectType> & _object_type, const boost::shared_ptr<ObjectPart> & _object_part, int & _perturbation);

		unsigned int computeId(const unsigned int & object_id, const unsigned int & object_part_id, const unsigned int & grasp_id, const unsigned int & grip_id, const unsigned int & reach_id, const unsigned int & perturbation_id);
};
#endif /* GRASP_H_ */
