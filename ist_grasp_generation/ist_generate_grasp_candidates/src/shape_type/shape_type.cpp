/*
 * shape_type.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: Rui P. Figueiredo
 */

#include "shape_type/shape_type.h"

std::map<unsigned int, boost::shared_ptr<ShapeType> > ShapeType::canonical_shapes;


ShapeType::ShapeType()
{
	// TODO Auto-generated constructor stub

}

ShapeType::~ShapeType() {
	// TODO Auto-generated destructor stub
}
