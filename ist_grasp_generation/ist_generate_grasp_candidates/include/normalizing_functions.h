/*
 * NormalizeFunctions.h
 *
 *  Created on: Jan 26, 2013
 *      Author: Rui P. Figueiredo
 */

#ifndef NORMALIZINGFUNCTIONS_H_
#define NORMALIZINGFUNCTIONS_H_

#include <iostream>

class NormalizingFunctions
{
	public:

		static double normalize(const double value,
									  const double minimum,
									  const double maximum);

		static double denormalize(const double normalized_value,
										const double minimum,
										const double maximum);

		NormalizingFunctions();
		virtual ~NormalizingFunctions();
};

#endif /* NORMALIZINGFUNCTIONS_H_ */
