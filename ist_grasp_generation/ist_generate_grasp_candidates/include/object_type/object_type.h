/*
 * object_type.h
 *
 *  Created on: Jan 23, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef OBJECTTYPE_H_
#define OBJECTTYPE_H_

#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <shape_type/shape_type.h>
#include <size_type/size_type.h>
#include <object_part/object_part.h>

class ObjectType
{
	public:
		static std::map<unsigned int, boost::shared_ptr<ObjectType> > object_types;

		unsigned int id;
		std::string name;

		// Shape type
		boost::shared_ptr<ShapeType> shape_type;

		// Size type
		boost::shared_ptr<SizeType> size_type;

		// Object parts
		std::vector<boost::shared_ptr<ObjectPart> > object_parts;

		static std::map<std::pair<int,int>,int> size_shape_object_map;
		static std::map<int,int> object_shape_map;

		ObjectType();
		ObjectType(unsigned int _id, std::string _name);

		virtual ~ObjectType();


		void update(const std::vector<Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> > _parts_pose, std::vector<Eigen::Vector3d> & _parts_dimensions)
		{
			for(unsigned int r=0; r < _parts_dimensions.size(); ++r)
			{
				object_parts[r]->update(_parts_pose[r], _parts_dimensions[r]);
			}
		}

		void getNormalizedParameters(double & _normalized_object_dimension_x,
								     double & _normalized_object_dimension_y,
									 double & _normalized_object_dimension_z)
		{
			// DUMMY
			_normalized_object_dimension_x=0.5;
			_normalized_object_dimension_y=0.5;
			_normalized_object_dimension_z=0.5;
		}

		void setParameters(const double _normalized_object_dimension_x,
						   const double _normalized_object_dimension_y,
						   const double _normalized_object_dimension_z)
		{
			// DUMMY

		}


};

#endif /* OBJECTTYPE_H_ */
