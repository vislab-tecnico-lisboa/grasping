/*
 * grip.h
 *
 *  Created on: Dec 14, 2012
 *      Author: rui
 */

#ifndef GRIP_H_
#define GRIP_H_

#include "canonical_grip/canonical_grip.h"

class Grip
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		boost::shared_ptr<CanonicalGrip> canonical_grip;

		// Grip pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, 3, Eigen::DontAlign> pose;

		static double palm_to_tip_distance;

		// Constructors
		Grip()
		{};

		Grip(const boost::shared_ptr<CanonicalGrip> _canonical_grip);

        void gripUpdate(const Eigen::Vector3d & dimensions);

		// Destructor
		virtual ~Grip();
};


#endif /* GRIP_H_ */
