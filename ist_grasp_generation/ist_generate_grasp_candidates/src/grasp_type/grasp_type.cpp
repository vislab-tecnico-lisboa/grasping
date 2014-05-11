/*
 * grasp_type.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: rui
 */

#include "grasp_type/grasp_type.h"

std::vector<GraspType> GraspType::grasp_types;

//const std::vector<GraspType> GraspType::grasp_types=boost::assign::list_of
//(GraspType(1))
//(GraspType(2))
//(GraspType(3));

GraspType::GraspType(int _type) : type(_type)
{
	if(type==1)
		type_name="Power";
	else if(type==2)
		type_name="Precision";
	else if(type==3)
		type_name="Intermediate";
}

