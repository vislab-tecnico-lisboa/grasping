/*
 * size_type.h
 *
 *  Created on: Jan 23, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef SIZETYPE_H_
#define SIZETYPE_H_

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

class SizeType
{
	public:
		static std::map<unsigned int, boost::shared_ptr<SizeType> > canonical_sizes;

		unsigned int id;

		std::string name;

		SizeType();
		virtual ~SizeType();
};

#endif /* SIZETYPE_H_ */
