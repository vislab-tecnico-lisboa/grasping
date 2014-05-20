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

void Grip::gripUpdate(const Eigen::Vector3d & dimensions)
{
    //Eigen::Vector3d  dimensions_temp(0.1,0.1,0.1);
    Eigen::Vector3d grip_trans(dimensions.array() * - canonical_grip->direction.normalized().array());

    // Use palm to grip distance to center grasp in the object
    /*if(grip_trans.norm()>palm_to_tip_distance)
    {
        //std::cout << "dimensao maior que grip:" << grip_trans.norm() << std::endl;
        grip_trans = grip_trans + (palm_to_tip_distance * canonical_grip->direction);
        //std::cout << "dimensao maior que grip after:" << grip_trans.norm() << std::endl;

    }
    else
    {
        //std::cout << "dimensao menor que grip:" << grip_trans.norm() << std::endl;
        grip_trans = Eigen::Vector3d(0.0,0.0,0.0);

    }*/
    pose=Eigen::Translation3d(grip_trans) * canonical_grip->orientation;
}

