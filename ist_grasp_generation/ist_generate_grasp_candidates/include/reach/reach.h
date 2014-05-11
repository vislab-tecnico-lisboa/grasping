/*
 * reach.h
 *
 *  Created on: Jan 22, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef REACH_H_
#define REACH_H_

#include "reach_type/reach_type.h"
#include "normalizing_functions.h"
// Continuous reach
class Reach
{
	public:
		// Discrete
		boost::shared_ptr<ReachType> reach_type;

		// Continuous
		double distance;

		Reach();

		Reach(boost::shared_ptr<ReachType> _reach_type);

		Reach(double _distance);

		virtual ~Reach();

		void getNormalizedParameters(double & _normalized_distance)
		{
			_normalized_distance=NormalizingFunctions::normalize(distance,reach_type->minimum_distance,reach_type->maximum_distance);
		}

		void setParameters(const double _normalized_distance)
		{
			distance=NormalizingFunctions::denormalize(_normalized_distance,reach_type->minimum_distance,reach_type->maximum_distance);
		}

		// Initialize
		void initialize();
};

#endif /* REACH_H_ */
