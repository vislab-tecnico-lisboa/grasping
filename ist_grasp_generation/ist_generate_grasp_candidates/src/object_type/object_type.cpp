/*
 * canonical_object.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: Rui P. Figueiredo
 */

#include "object_type/object_type.h"

std::map<unsigned int, boost::shared_ptr<ObjectType> > ObjectType::object_types;

ObjectType::ObjectType(unsigned int _id, std::string _name) :id(_id), name(_name)
{

}


ObjectType::ObjectType()
{
	// TODO Auto-generated constructor stub
}

ObjectType::~ObjectType()
{
	// TODO Auto-generated destructor stub
}
