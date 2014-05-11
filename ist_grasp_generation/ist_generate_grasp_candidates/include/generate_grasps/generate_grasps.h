/*
 * generate_grasps.h
 *
 *  Created on: Dec 15, 2012
 *      Author: rui
 */

#ifndef GENERATEGRASPS_H_
#define GENERATEGRASPS_H_

#include "grasp/grasp.h"
#include "grasp_perturbation/grasp_perturbation.h"
#include <map>

class GenerateGrasps
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		GenerateGrasps();
		virtual ~GenerateGrasps();

		std::map<unsigned int, boost::shared_ptr<Grasp> > generated_grasps;
};

#endif /* GENERATEGRASPS_H_ */
