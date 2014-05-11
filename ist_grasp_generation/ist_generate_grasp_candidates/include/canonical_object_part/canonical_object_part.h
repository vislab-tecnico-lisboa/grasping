/*
 * CanonicalObjectPart.h
 *
 *  Created on: Mar 15, 2013
 *      Author: rui
 */

#ifndef CANONICALOBJECTPART_H_
#define CANONICALOBJECTPART_H_

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

class CanonicalObjectPart
{
	public:
		// List containing all possible canonical object parts
		static std::map<unsigned int, boost::shared_ptr<CanonicalObjectPart> > canonical_object_parts;

		unsigned int id;

		std::string name;

		CanonicalObjectPart(const unsigned int & _id) : id(_id), name(canonical_object_parts.find(_id)->second->name)
		{

			//std::cout << " TESTE!!!:" << canonical_object_parts.find(_id)->second->name << std::endl;
		};

		CanonicalObjectPart(const unsigned int & _id, const std::string & _name) : id(_id), name(_name)
		{};

		CanonicalObjectPart();

		virtual ~CanonicalObjectPart();
};

#endif /* CANONICALOBJECTPART_H_ */
