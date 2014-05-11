/*
 * grip.h
 *
 *  Created on: Dec 14, 2012
 *      Author: Rui P. de Figueiredo
 */

#ifndef CANONICALGRIP_H_
#define CANONICALGRIP_H_

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <boost/assign/list_of.hpp> // for 'list_of()'
#include <boost/assert.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/StdVector>
#include <map>
#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#define RIGHT_GRIP  (type>=1 &&type<=4 )
#define FRONT_GRIP  (type>=5 &&type<=8 )
#define LEFT_GRIP   (type>=9 &&type<=12)
#define BACK_GRIP   (type>=13&&type<=16)
#define TOP_GRIP    (type>=17&&type<=20)
#define BOTTOM_GRIP (type>=21&&type<=24)

class CanonicalGrip
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Static definitions (object coordinate axis)
		static const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> x_o;
		static const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> y_o;
		static const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> z_o;

		// List containing all possible canonical grips
//  		static const std::map<unsigned int, boost::shared_ptr<CanonicalGrip> > canonical_grips;
		static std::map<unsigned int, boost::shared_ptr<CanonicalGrip> > canonical_grips;

		// Grip type (discrete) (1 to 24)
		int type;

		// Name of the grip type
		std::string grip_name;

		// Grip direction (continuous)
		Eigen::Matrix<double, 3, 1, Eigen::DontAlign> direction;

		// Grip canonical orientation (object coordinate frame)
		Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> orientation;



		// Constructors
		CanonicalGrip()
		{};

		CanonicalGrip(const int & _type);
		CanonicalGrip(const int & _type, const std::string & _name, Eigen::Matrix<double, 3, 1> & _direction, Eigen::Transform<double, 3, Eigen::Affine> & _orientation);


		// Destructor
		virtual ~CanonicalGrip()
		{};

		// Method that computes grip poses
		//boost::shared_ptr<std::vector<Eigen::Affine3d> > generateGripPoses(const Eigen::Vector3d & dimensions);

		// Method that computes pre grip poses
		//boost::shared_ptr<std::vector<Eigen::Affine3d> > generatePreGripPoses(const Eigen::Vector3d & dimensions);
};

#endif /* CANONICALGRIP_H_ */
