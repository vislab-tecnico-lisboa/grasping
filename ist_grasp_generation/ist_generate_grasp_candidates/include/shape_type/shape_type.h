/*
 * shape_type.h
 *
 *  Created on: Jan 23, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef SHAPETYPE_H_
#define SHAPETYPE_H_

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

class ShapeType
{
	public:
		static std::map<unsigned int, boost::shared_ptr<ShapeType> > canonical_shapes;

		unsigned id;
		std::string name;

		ShapeType();
		virtual ~ShapeType();
};

#endif /* SHAPETYPE_H_ */
