/*
 * joint_state.h
 *
 *  Created on: Dec 16, 2012
 *      Author: rui
 */

#ifndef JOINTSTATE_H_
#define JOINTSTATE_H_

#include <iostream>
#include <vector>

class JointState
{
	public:
		// Attributes
		std::string name;
		double position;
		double velocity;
		double effort;

		// Constructors
		JointState();

		JointState(std::string _name, double _position=0.0, double _velocity=0.0, double _effort=0.0) : name(_name), position(_position), velocity(_velocity), effort(_effort)
		{};

		// Destructor
		virtual ~JointState();
};

#endif /* JOINTSTATE_H_ */
