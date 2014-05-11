/*
 * definitions.h
 *
 *  Created on: Jan 26, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#define MAX_X_OFFSET 0.06
#define MAX_Y_OFFSET 0.06
#define MAX_Z_OFFSET 0.06
#define MAX_YAW 45
#define MAX_PITCH 45
#define MAX_ROLL 45

#define PI 3.14159265
#define DEG_TO_RAD    ((PI)/(180))
#define MAX_YAW_RAD   (MAX_YAW*DEG_TO_RAD)
#define MAX_PITCH_RAD (MAX_PITCH*DEG_TO_RAD)
#define MAX_ROLL_RAD  (MAX_ROLL*DEG_TO_RAD)

#endif /* DEFINITIONS_H_ */
