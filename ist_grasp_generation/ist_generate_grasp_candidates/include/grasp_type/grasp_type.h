/*
 * grasp_type.h
 *
 *  Created on: Dec 15, 2012
 *      Author: rui
 */

#ifndef GRASP_TYPE_H_
#define GRASP_TYPE_H_

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <boost/assign/list_of.hpp> // for 'list_of()'
#include <boost/assert.hpp>

class GraspType
{
	public:
		// List containing all possible grasp types
		//static const std::vector<GraspType> grasp_types;
		static std::vector<GraspType> grasp_types;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Grasp type (discrete) (1 to 3)
		int type;

		// Name of the grasp type
		std::string type_name;

		// Constructors
		GraspType()
		{};

		GraspType(int _type);

		GraspType(int _type, std::string & _type_name) : type(_type), type_name(_type_name)
		{}
};

#endif /* GRASP_TYPE_H_ */
