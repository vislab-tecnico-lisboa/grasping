/*
 * reach_type.h
 *
 *  Created on: Jan 22, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef REACH_TYPE_H_
#define REACH_TYPE_H_

#define EIGEN_USE_NEW_STDVECTOR
#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/StdVector>
#include <map>
#include <boost/shared_ptr.hpp>
class ReachType
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		static std::map<unsigned int, boost::shared_ptr<ReachType> > reach_types;

		unsigned int id;
		std::string name;

		// Fixed continuous parameters
		double minimum_distance;
		double maximum_distance;

		Eigen::Vector3d direction;

		// Constructors

		ReachType()
		{};

		ReachType(int _id) : id(_id)
		{};

  		ReachType(unsigned int _id, std::string _name, double _minimum_distance, double _maximum_distance, Eigen::Vector3d & _direction) : id(_id), name(_name), minimum_distance(_minimum_distance), maximum_distance(_maximum_distance), direction(_direction)
		{};

};

#endif /* REACH_TYPE_H_ */
