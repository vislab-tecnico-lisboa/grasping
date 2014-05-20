 /*
 * grasp.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: Rui P. Figueiredo
 */

#include "grasp/grasp.h"

Grasp::Grasp()
{}

Grasp::Grasp(const boost::shared_ptr<CanonicalGrasp> & _canonical_grasp, const Reach & _reach, const Grip & _grip, const boost::shared_ptr<ObjectType> & _object_type, const boost::shared_ptr<ObjectPart> & _object_part, int & _perturbation) :
		canonical_grasp(_canonical_grasp),
		reach(_reach),
		grip(_grip),
		object_type(_object_type),
		object_part(_object_part),
		//offset_rotation(canonical_grasp->pose_offset),
		//offset_translation(canonical_grasp->pose_offset.translation()),
        grasp_perturbation(_perturbation, canonical_grasp->perturbation_angle_step, canonical_grasp->perturbation_rotation_axis)

{
    id=computeId(object_type->id, object_part->id, canonical_grasp->number, grip.canonical_grip->type, reach.reach_type->id, grasp_perturbation.perturbation+canonical_grasp->n_perturbs);

    ///////////////////////////////
	// GRASP AND PRE GRASP POSES //
	///////////////////////////////

	Eigen::Vector3d pre_grasp_position(canonical_grasp->pre_grasp_distance, canonical_grasp->pre_grasp_distance, canonical_grasp->pre_grasp_distance);
	Eigen::Vector3d grasp_reach_position(reach.distance, reach.distance, reach.distance);

	Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pre_grasp_position_(Eigen::Translation3d(pre_grasp_position.array() * - grip.canonical_grip->direction.array()));
	Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> reach_position_(Eigen::Translation3d(grasp_reach_position.array() * - grip.canonical_grip->direction.array()));


    // Transform from handle frame definition to ros frame definition
    Eigen::Matrix3d handle_to_ros_palm_rotation_matrix;
    handle_to_ros_palm_rotation_matrix << 1, 0, 0,
                                          0, 0,-1,
                                          0, 1, 0;
    hand_canonical_pose=Eigen::Translation3d(0.0 ,0.0 , 0.0)*Eigen::AngleAxisd(handle_to_ros_palm_rotation_matrix);
    //Eigen::Transform<double, 3, Eigen::Affine> ros_palm_to_handle_transform=handle_to_ros_palm_transform.inverse();
    //Eigen::Transform<double, 3, Eigen::Affine> grasp_pose=grasp->pose*ros_palm_to_handle_transform;

    grasp_object_center_pose    =canonical_grasp->pose_offset*canonical_grasp->orientation*grasp_perturbation.orientation*hand_canonical_pose.inverse();
	grasp_object_center_pre_pose=pre_grasp_position_*grasp_object_center_pose;

    pose=reach_position_*grip.pose*grasp_object_center_pose;
    pre_pose=pre_grasp_position_*pose;
}



void Grasp::normalizedEulerToEigen(const double _normalized_yaw,
								   const double _normalized_pitch,
								   const double _normalized_roll,
								   Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> & _rotation)
{
	double yaw, pitch, roll;
	yaw=_normalized_yaw*2*MAX_YAW_RAD - MAX_YAW_RAD;
	roll=_normalized_roll*2*MAX_ROLL_RAD - MAX_ROLL_RAD;
	pitch=_normalized_pitch*2*MAX_PITCH_RAD - MAX_PITCH_RAD;

	_rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void Grasp::eigenToEuler(const Eigen::Transform<double, 3, Eigen::Affine> & transform,
						 double & _yaw,
						 double & _pitch,
						 double & _roll)
{
	Eigen::Vector3d ea = transform.rotation().matrix().eulerAngles(0, 1, 2);
	_roll=ea[0];
	_pitch=ea[1];
	_yaw=ea[2];
}

void Grasp::getNormalizedYawPitchRoll(const Eigen::Transform<double, 3, Eigen::Affine> & _rotation,
									  double & _normalized_yaw,
									  double & _normalized_pitch,
									  double & _normalized_roll)
{
	double yaw, pitch, roll;
	eigenToEuler(_rotation, yaw, pitch, roll);
	_normalized_yaw=	NormalizingFunctions::normalize(yaw,	-MAX_YAW_RAD,	MAX_YAW_RAD);
	_normalized_pitch=	NormalizingFunctions::normalize(pitch,	-MAX_PITCH_RAD,	MAX_PITCH_RAD);
	_normalized_roll=	NormalizingFunctions::normalize(roll,	-MAX_ROLL_RAD,	MAX_ROLL_RAD);
}

void Grasp::getNormalizedTranslation(const Eigen::Transform<double, 3, Eigen::Affine> & _translation,
									 double & _normalized_x,
									 double & _normalized_y,
									 double & _normalized_z)
{
	// Normalize x
	_normalized_x=NormalizingFunctions::normalize(_translation.translation().x(),-MAX_X_OFFSET,MAX_X_OFFSET);
	_normalized_y=NormalizingFunctions::normalize(_translation.translation().y(),-MAX_Y_OFFSET,MAX_Y_OFFSET);
	_normalized_z=NormalizingFunctions::normalize(_translation.translation().z(),-MAX_Z_OFFSET,MAX_Z_OFFSET);
}

void Grasp::denormalizedTranslation(const double _normalized_x,
									const double _normalized_y,
								    const double _normalized_z,
								    Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> & _translation)
{
	double _translation_x=NormalizingFunctions::denormalize(_normalized_x,-MAX_X_OFFSET,MAX_X_OFFSET);
	double _translation_y=NormalizingFunctions::denormalize(_normalized_y,-MAX_Y_OFFSET,MAX_Y_OFFSET);
	double _translation_z=NormalizingFunctions::denormalize(_normalized_z,-MAX_Z_OFFSET,MAX_Z_OFFSET);

	_translation=Eigen::Translation3d(_translation_x, _translation_y, _translation_z);
}

void Grasp::getNormalizedParameters(double & _normalized_yaw,
		                            double & _normalized_pitch,
		                            double & _normalized_roll,
		                            double & _normalized_x,
		                            double & _normalized_y,
		                            double & _normalized_z,
		                            double & _normalized_reach_distance,
		                            double & _normalized_object_dimension_x,
		                           	double & _normalized_object_dimension_y,
		                           	double & _normalized_object_dimension_z)
{
	getNormalizedYawPitchRoll(offset_rotation, _normalized_yaw, _normalized_pitch, _normalized_roll);
	getNormalizedTranslation(offset_translation, _normalized_x, _normalized_y, _normalized_z);
	reach.getNormalizedParameters(_normalized_reach_distance);
	object_type->getNormalizedParameters(_normalized_object_dimension_x,_normalized_object_dimension_y,_normalized_object_dimension_z);
	//std::cout << "DEBUG NORMALIZATION:" << _normalized_yaw << " " << _normalized_pitch << " " << _normalized_roll << " " << _normalized_x << " " << _normalized_y << " " << _normalized_z << " " << _normalized_reach_distance << " " << _normalized_object_dimension_x << " " << _normalized_object_dimension_y << " " << _normalized_object_dimension_z << std::endl;
}

void Grasp::setParameters(const double _normalized_yaw,
						  const double _normalized_pitch,
						  const double _normalized_roll,
						  const double _normalized_x,
						  const double _normalized_y,
                          const double _normalized_z,
						  const double _normalized_reach_distance,
						  const double _normalized_object_dimension_x,
						  const double _normalized_object_dimension_y,
						  const double _normalized_object_dimension_z)
{
	normalizedEulerToEigen(_normalized_yaw, _normalized_pitch, _normalized_roll, offset_rotation);
	denormalizedTranslation(_normalized_x, _normalized_y, _normalized_z, offset_translation);
	reach.setParameters(_normalized_reach_distance);
	object_type->setParameters(_normalized_object_dimension_x,_normalized_object_dimension_y,_normalized_object_dimension_z);
}

void Grasp::updateGrasp(const Eigen::Vector3d & _object_part_dimensions, const Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> & _object_part_pose)
{
    grip.gripUpdate(_object_part_dimensions);


    Eigen::Vector3d pre_grasp_position(canonical_grasp->pre_grasp_distance, canonical_grasp->pre_grasp_distance, canonical_grasp->pre_grasp_distance);
    Eigen::Vector3d grasp_reach_position(reach.distance, reach.distance, reach.distance);

    Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pre_grasp_position_(Eigen::Translation3d(pre_grasp_position.array() * - grip.canonical_grip->direction.array()));
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> reach_position_(Eigen::Translation3d(grasp_reach_position.array() * - grip.canonical_grip->direction.array()));


    pose=_object_part_pose*reach_position_*grip.pose*grasp_object_center_pose;
    pre_pose=pre_grasp_position_*pose;
}

unsigned int Grasp::computeId(const unsigned int & object_id, const unsigned int & object_part_id, const unsigned int & grasp_id, const unsigned int & grip_id, const unsigned int & reach_id, const unsigned int & perturbation_id)
{
	return  (object_id-1 +
			(object_part_id-1)*ObjectType::object_types.size() +
			(grasp_id-1)      *ObjectType::object_types.size()*CanonicalObjectPart::canonical_object_parts.size() +
			(grip_id-1)       *ObjectType::object_types.size()*CanonicalObjectPart::canonical_object_parts.size()*CanonicalGrasp::canonical_grasps.size()+
			(reach_id-1)      *ObjectType::object_types.size()*CanonicalObjectPart::canonical_object_parts.size()*CanonicalGrasp::canonical_grasps.size()*CanonicalGrip::canonical_grips.size())+
			perturbation_id   *ObjectType::object_types.size()*CanonicalObjectPart::canonical_object_parts.size()*CanonicalGrasp::canonical_grasps.size()*CanonicalGrip::canonical_grips.size()*ReachType::reach_types.size();
}
