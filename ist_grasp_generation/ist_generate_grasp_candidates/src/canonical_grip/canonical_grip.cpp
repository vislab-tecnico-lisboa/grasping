/*
 * grip.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: rui
 */

#include <canonical_grip/canonical_grip.h>

const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> CanonicalGrip::x_o = Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign>(1,0,0);
const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> CanonicalGrip::y_o = Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign>(0,1,0);
const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> CanonicalGrip::z_o = Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign>(0,0,1);

std::map<unsigned int, boost::shared_ptr<CanonicalGrip> > CanonicalGrip::canonical_grips;


//const std::map<unsigned int, boost::shared_ptr<CanonicalGrip> > CanonicalGrip::canonical_grips=boost::assign::map_list_of
//( 1, boost::make_shared<CanonicalGrip>(CanonicalGrip(1)))  // 1. right grip thumb-front
//( 2, boost::make_shared<CanonicalGrip>(CanonicalGrip(2)))  // 2. right grip thumb-up
////( 3, boost::make_shared<canonicalgrip>(canonicalgrip(3)))  // 3. right grip thumb-back
////( 4, boost::make_shared<canonicalgrip>(canonicalgrip(4)))  // 4. right grip thumb-down
//( 5, boost::make_shared<CanonicalGrip>(CanonicalGrip(5)))  // 5. front grip thumb-left
//( 6, boost::make_shared<CanonicalGrip>(CanonicalGrip(6)))  // 6. front grip thumb-up
////( 7, boost::make_shared<CanonicalGrip>(CanonicalGrip(7)))  // 7. front grip thumb-right
////( 8, boost::make_shared<CanonicalGrip>(CanonicalGrip(8)))  // 8. front grip thumb-down
//( 9, boost::make_shared<CanonicalGrip>(CanonicalGrip(9)))  // 9. left grip thumb-back
//(10, boost::make_shared<CanonicalGrip>(CanonicalGrip(10))) // 10. left grip thumb-up
////(11, boost::make_shared<CanonicalGrip>(CanonicalGrip(11))) // 11. left grip thumb-front
////(12, boost::make_shared<CanonicalGrip>(CanonicalGrip(12))) // 12. left grip thumb-down
//(13, boost::make_shared<CanonicalGrip>(CanonicalGrip(13))) // 13. back grip thumb-right
//(14, boost::make_shared<CanonicalGrip>(CanonicalGrip(14))) // 14. back grip thumb-up
////(15, boost::make_shared<CanonicalGrip>(CanonicalGrip(15))) // 15. back grip thumb-left
////(16, boost::make_shared<CanonicalGrip>(CanonicalGrip(16))) // 16. back grip thumb-down
//(17, boost::make_shared<CanonicalGrip>(CanonicalGrip(17)))// 17. top grip thumb-front
//(18, boost::make_shared<CanonicalGrip>(CanonicalGrip(18))) // 18. top grip thumb-left
////(19, boost::make_shared<CanonicalGrip>(CanonicalGrip(19))) // 19. top grip thumb-back
////(20, boost::make_shared<CanonicalGrip>(CanonicalGrip(20))) // 20. top grip thumb-right
//(21, boost::make_shared<CanonicalGrip>(CanonicalGrip(21))) // 21. bottom grip thumb-front
//(22, boost::make_shared<CanonicalGrip>(CanonicalGrip(22)));// 22. bottom grip thumb-right
////(23, boost::make_shared<CanonicalGrip>(CanonicalGrip(23))) // 23. bottom grip thumb-back
////(24, boost::make_shared<CanonicalGrip>(CanonicalGrip(24)));// 24. bottom grip thumb-left;

CanonicalGrip::CanonicalGrip(const int & _type) : type(_type)
{
	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> rotation_matrix;
	/////////////////////
	// Grips direction //
	/////////////////////

	if(RIGHT_GRIP)
	{
		direction=y_o;
	}
	else if(FRONT_GRIP)
	{
		direction=x_o;
	}
	else if(LEFT_GRIP)
	{
		direction=-y_o;
	}
	else if(BACK_GRIP)
	{
		direction=-x_o;
	}
	else if(TOP_GRIP)
	{
		direction=-z_o;
	}
	else if(BOTTOM_GRIP)
	{
		direction=z_o;
	}

	if(type==1)
	{
		// 1. right grip thumb-front
		rotation_matrix <<  x_o,  y_o,  z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="right grip thumb-front";
	}
	else if(type==2)
	{
		// 2. right grip thumb-up
		rotation_matrix << -z_o,  y_o,  x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="right grip thumb-up";
	}
	else if(type==3)
	{
		// 3. right grip thumb-back
		rotation_matrix << -x_o,  y_o, -z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="right grip thumb-back";
	}
	else if(type==4)
	{
		// 4. right grip thumb-down
		rotation_matrix <<  z_o,  y_o, -x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="right grip thumb-down";
	}

	/////////////////
	// Front grips //
	/////////////////

	else if(type==5)
	{
		// 5. front grip thumb-left
		rotation_matrix << -y_o,  x_o,  z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="front grip thumb-left";
	}
	else if(type==6)
	{
		// 6. front grip thumb-up
		rotation_matrix << -z_o,  x_o, -y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="front grip thumb-up";
	}
	else if(type==7)
	{
		// 7. front grip thumb-right
		rotation_matrix <<  y_o,  x_o, -z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="front grip thumb-right";

	}
	else if(type==8)
	{
		// 8. front grip thumb-down
		rotation_matrix <<  z_o,  x_o,  y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="front grip thumb-down";
	}

	////////////////
	// Left grips //
	////////////////

	else if(type==9)
	{
		// 9. left grip thumb-back
		rotation_matrix << -x_o, -y_o,  z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="left grip thumb-back";
	}
	else if(type==10)
	{
		// 10. left grip thumb-up
		rotation_matrix << -z_o, -y_o, -x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="left grip thumb-up";
	}
	else if(type==11)
	{
		// 11. left grip thumb-front
		rotation_matrix <<  x_o, -y_o, -z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="left grip thumb-front";
	}
	else if(type==12)
	{
		// 12. left grip thumb-down
		rotation_matrix <<  z_o, -y_o,  x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="left grip thumb-down";
	}

	////////////////
	// Back grips //
	////////////////

	else if(type==13)
	{
		// 13. back grip thumb-right
		rotation_matrix <<  y_o, -x_o,  z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="back grip thumb-right";
	}
	else if(type==14)
	{
		// 14. back grip thumb-up
		rotation_matrix << -z_o, -x_o,  y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="back grip thumb-up";
	}
	else if(type==15)
	{
		// 15. back grip thumb-left
		rotation_matrix << -y_o, -x_o, -z_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="back grip thumb-left";
	}
	else if(type==16)
	{
		// 16. back grip thumb-down
		rotation_matrix <<  z_o, -x_o, -y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="back grip thumb-down";
	}

	///////////////
	// Top grips //
	///////////////

	else if(type==17)
	{
		// 17. top grip thumb-front
		rotation_matrix <<  x_o, -z_o,  y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="top grip thumb-front";
	}
	else if(type==18)
	{
		// 18. top grip thumb-left
		rotation_matrix << -y_o, -z_o,  x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="top grip thumb-left";
	}
	else if(type==19)
	{
		// 19. top grip thumb-back
		rotation_matrix << -x_o, -z_o, -y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="top grip thumb-back";
	}
	else if(type==20)
	{
		// 20. top grip thumb-right
		rotation_matrix <<  y_o, -z_o, -x_o;

		//std::cout << rotation_matrix << std::endl;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);

		Eigen::Quaternion<double> quat(rotation_matrix);
		//std::cout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "  << std::endl;
		grip_name="top grip thumb-right";
		//exit(1);
	}

	//////////////////
	// Bottom grips //
	//////////////////

	else if(type==21)
	{
		// 21. bottom grip thumb-front
		rotation_matrix <<  x_o,  z_o, -y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="bottom grip thumb-front";
	}
	else if(type==22)
	{
		// 22. bottom grip thumb-right
		rotation_matrix <<  y_o,  z_o,  x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="bottom grip thumb-right";
	}
	else if(type==23)
	{
		// 23. bottom grip thumb-back
		rotation_matrix << -x_o,  z_o,  y_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="bottom grip thumb-back";
	}
	else if(type==24)
	{
		// 24. bottom grip thumb-left
		rotation_matrix << -y_o,  z_o, -x_o;
		orientation=Eigen::Transform<double, 3, 3, Eigen::DontAlign>(rotation_matrix);
		grip_name="bottom grip thumb-left";
	}
	else
	{
		std::cout << "TYPE:" << type << "INHEXISTENT GRIP TYPE!" << std::endl;
		//exit(-1);
	}
}

CanonicalGrip::CanonicalGrip(const int & _type, const std::string & _name, Eigen::Matrix<double, 3, 1> & _direction, Eigen::Transform<double, 3, Eigen::Affine> & _orientation) :
		type(_type), grip_name(_name), direction(_direction), orientation(_orientation)
{}
