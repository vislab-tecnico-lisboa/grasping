/*
 * grip.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: rui
 */

#include "grip/grip.h"

double Grip::palm_to_tip_distance;

Grip::Grip(boost::shared_ptr<CanonicalGrip> _canonical_grip) : canonical_grip(_canonical_grip)
{
	pose=canonical_grip->orientation;
}

Grip::~Grip()
{

}

