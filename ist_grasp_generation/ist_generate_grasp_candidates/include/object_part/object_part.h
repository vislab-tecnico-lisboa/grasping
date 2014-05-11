/*
 * ObjectPart.h
 *
 *  Created on: Mar 15, 2013
 *      Author: Rui
 */

#ifndef OBJECTPART_H_
#define OBJECTPART_H_

#define EIGEN_USE_NEW_STDVECTOR
#define EIGEN_DONT_ALIGN_STATICALLY

#include <Eigen/StdVector>

#include "canonical_object_part/canonical_object_part.h"
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>

class ObjectPart : public CanonicalObjectPart
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		// Object part pose in the object coordinate frame (continuous)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pose;

		Eigen::Vector3d dimensions;

		ObjectPart(const unsigned int & _id) : CanonicalObjectPart(_id)
		{};

		ObjectPart(const unsigned int & _id, const std::string & _name) : CanonicalObjectPart(_id,_name)
		{};

		ObjectPart();
		virtual ~ObjectPart();

		void update(const Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> _pose, Eigen::Vector3d & _dimensions)
		{
			pose=_pose;
			dimensions=_dimensions;
		}
};

#endif /* OBJECTPART_H_ */
