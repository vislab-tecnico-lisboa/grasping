/*
 * GraspPerturbation.cpp
 *
 *  Created on: Jan 26, 2013
 *      Author: rui
 */

#include "grasp_perturbation/grasp_perturbation.h"

GraspPerturbation::GraspPerturbation()
{}


GraspPerturbation::GraspPerturbation(int & _perturbation, double & _angle_step, Eigen::Vector3d & _rotation_axis): perturbation(_perturbation), rotation_angle(_angle_step*perturbation)
{
	orientation=Eigen::AngleAxisd(rotation_angle, _rotation_axis)*Eigen::Translation3d(0.0,0.0,0.0);
}

GraspPerturbation::~GraspPerturbation()
{
	// TODO Auto-generated destructor stub
}


void GraspPerturbation::updateGraspPerturbation(const Eigen::Vector3d & dimensions)
{}
