/*
 * hand_posture.h
 *
 *  Created on: Dec 16, 2012
 *      Author: Rui P. Figueiredo
 */

#ifndef HANDPOSTURE_H_
#define HANDPOSTURE_H_

#include "joint_state/joint_state.h"

class HandPosture
{
	public:
		std::vector<JointState> joint_states;
		HandPosture();
		virtual ~HandPosture();
};

#endif /* HANDPOSTURE_H_ */
