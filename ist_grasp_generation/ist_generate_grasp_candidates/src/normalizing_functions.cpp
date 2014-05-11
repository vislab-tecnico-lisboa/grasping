/*
 * normalizing_functions.cpp
 *
 *  Created on: Jan 26, 2013
 *      Author: Rui P. Figueiredo
 */

#include "normalizing_functions.h"

NormalizingFunctions::NormalizingFunctions() {
	// TODO Auto-generated constructor stub

}

NormalizingFunctions::~NormalizingFunctions() {
	// TODO Auto-generated destructor stub
}

double NormalizingFunctions::normalize(const double value,
											 const double minimum,
											 const double maximum)
{
	if(maximum-minimum>0.00001 && value >= minimum && value<=maximum)
	{
		//std::cout << " normalized value: " << (value-minimum)/(maximum-minimum) << std::endl;

		return (value-minimum)/(maximum-minimum);
	}
	//std::cout << "minimum: " <<minimum << " maximum: " << maximum  << " value: " << value << std::endl;

	//std::cout << "INPUT VALUE OUT OF BOUNDS WHEN NORMALIZING" << std::endl;
	return 0.5;
}

double NormalizingFunctions::denormalize(const double normalized_value,
											   const double minimum,
											   const double maximum)
{
	if(maximum-minimum>0.00001 && normalized_value >= 0.0 && normalized_value<=1.0)
	{
		return (normalized_value)*(maximum-minimum)+minimum;
	}
	//std::cout << "MINIMUM: " <<minimum << " MAXIMUM: " << maximum << std::endl;
	//std::cout << "INPUT VALUE OUT OF BOUNDS WHEN DENORMALIZING" << " VALUE: " << normalized_value << std::endl;
	return maximum-minimum;
}
