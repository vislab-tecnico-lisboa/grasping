/*
 * grasp_perturbation.h
 *
 *  Created on: Jan 26, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef GRASPPERTURBATION_H_
#define GRASPPERTURBATION_H_

#include <Eigen/Eigen>

class GraspPerturbation
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int perturbation; //[perturbation + n_perturbations]
		double rotation_angle;

		// Perturbation pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> orientation;

		GraspPerturbation();

		GraspPerturbation(int & perturbation, double & _rotation_angle_step, Eigen::Vector3d & _rotation_axis);

		virtual ~GraspPerturbation();

		void updateGraspPerturbation(const Eigen::Vector3d & dimensions);
};

#endif /* GRASPPERTURBATION_H_ */
