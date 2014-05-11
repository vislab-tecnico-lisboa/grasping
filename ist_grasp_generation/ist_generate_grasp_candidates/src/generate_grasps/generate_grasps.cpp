/*
 * generate_grasps.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: rui
 */

#include "generate_grasps/generate_grasps.h"

// initializes Grasp::pre_grasp_distance
GenerateGrasps::GenerateGrasps()
{
	/////////////////////////////////////////////////////////////////////////////
	// Generate one grasp for each canonical grasp and for each canonical grip //
	/////////////////////////////////////////////////////////////////////////////
	int teste=0;
	std::cout << "reach_types:" << ReachType::reach_types.size() << std::endl;
	std::cout << "canonical_grips:" << CanonicalGrip::canonical_grips.size() << std::endl;
	std::cout << "CanonicalGrasp:" << CanonicalGrasp::canonical_grasps.size() << std::endl;
	std::cout << "object_types:" << ObjectType::object_types.size() << std::endl;

	generated_grasps.clear();

	for(std::map<unsigned int, boost::shared_ptr<ReachType> >::iterator reach_type_it=ReachType::reach_types.begin(); reach_type_it!=ReachType::reach_types.end(); ++reach_type_it)
	{
		for(std::map<unsigned int, boost::shared_ptr<CanonicalGrip> >::const_iterator canonical_grip_it=CanonicalGrip::canonical_grips.begin(); canonical_grip_it!=CanonicalGrip::canonical_grips.end(); ++canonical_grip_it)
		{
			for(std::map<unsigned int, boost::shared_ptr<CanonicalGrasp> >::iterator canonical_grasp_it=CanonicalGrasp::canonical_grasps.begin(); canonical_grasp_it!=CanonicalGrasp::canonical_grasps.end(); ++canonical_grasp_it)
			{
				for(std::map<unsigned int, boost::shared_ptr<ObjectType> >::iterator object_type_it=ObjectType::object_types.begin(); object_type_it!=ObjectType::object_types.end(); ++object_type_it)
				{
					for(std::vector<boost::shared_ptr<ObjectPart> >::iterator object_part_it=object_type_it->second->object_parts.begin(); object_part_it!=object_type_it->second->object_parts.end(); ++object_part_it)
					{
						for(int perturb=(int)(-canonical_grasp_it->second->n_perturbs); perturb <=(int)canonical_grasp_it->second->n_perturbs;++perturb)
						{
							boost::shared_ptr<Grasp> new_grasp(new  Grasp(	canonical_grasp_it->second,
																			Reach(boost::shared_ptr<ReachType>(reach_type_it->second)),
																			Grip(canonical_grip_it->second),
																			object_type_it->second,
																			*object_part_it,
																			perturb));

							generated_grasps.insert(std::pair<unsigned int, boost::shared_ptr<Grasp> >(new_grasp->id, new_grasp));
						}
					}
				}
			}
		}
	}

	std::cout << "GENERATED GRASPS: " << generated_grasps.size() << std::endl;
	//exit(1);
}

GenerateGrasps::~GenerateGrasps()
{
	// TODO Auto-generated destructor stub
}

