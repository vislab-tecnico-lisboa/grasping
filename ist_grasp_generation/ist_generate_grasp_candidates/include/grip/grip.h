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

		Eigen::Transform<double, 3, 3, Eigen::DontAlign> gripPose(unsigned int grip_id, const Eigen::Vector3d & dimensions)
		{
			Eigen::Transform<double, 3, 3, Eigen::DontAlign> pose_;
			canonical_grip=CanonicalGrip::canonical_grips.find(grip_id)->second;
			Eigen::Vector3d grip_trans(dimensions.array() * - canonical_grip->direction.normalized().array());

			// Use palm to grip distance to center grasp in the object
			if(grip_trans.norm()>palm_to_tip_distance)
			{
				//std::cout << "dimensao maior que grip:" << grip_trans.norm() << std::endl;
				grip_trans = grip_trans + (palm_to_tip_distance * canonical_grip->direction);
				//std::cout << "dimensao maior que grip after:" << grip_trans.norm() << std::endl; 

			}
			else
			{
				//std::cout << "dimensao menor que grip:" << grip_trans.norm() << std::endl;
				grip_trans = Eigen::Vector3d(0.0,0.0,0.0);

			}
			pose_=Eigen::Translation3d(grip_trans) * canonical_grip->orientation;
			return pose_;

		}

		// Destructor
		virtual ~Grip();
};


#endif /* GRIP_H_ */
