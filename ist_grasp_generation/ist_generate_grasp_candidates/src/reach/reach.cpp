/*
 * reach.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Rui P. Figueiredo
 */

#include "reach/reach.h"

Reach::Reach()
{

}

Reach::Reach(boost::shared_ptr<ReachType> _reach_type) : reach_type(_reach_type)
{
	initialize();
}

Reach::Reach(double _distance) : distance(_distance)
{}

Reach::~Reach() {
	// TODO Auto-generated destructor stub
}

// Initialize
void Reach::initialize()
{
	distance=(reach_type->maximum_distance+reach_type->minimum_distance)/2;
}
