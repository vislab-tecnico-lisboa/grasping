/*
 * grasp.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: rui
 */

#include <canonical_grasp/canonical_grasp.h>

const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> CanonicalGrasp::x_g = Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign>(1,0,0);
const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> CanonicalGrasp::y_g = Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign>(0,1,0);
const Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign> CanonicalGrasp::z_g = Eigen::Matrix<double, 3 ,1 , Eigen::DontAlign>(0,0,1);

std::map<unsigned int, boost::shared_ptr<CanonicalGrasp> > CanonicalGrasp::canonical_grasps;

//const std::vector<CanonicalGrasp,Eigen::aligned_allocator<CanonicalGrasp> > CanonicalGrasp::canonical_grasps=boost::assign::list_of
//(CanonicalGrasp( 1, 1))
//(CanonicalGrasp( 1, 2))
//(CanonicalGrasp( 1, 3))
//(CanonicalGrasp( 1, 4))
//(CanonicalGrasp( 1, 5))
//(CanonicalGrasp( 2, 6))
//(CanonicalGrasp( 2, 7))
//(CanonicalGrasp( 2, 8))
//(CanonicalGrasp( 2, 9))
//(CanonicalGrasp( 1,10))
//(CanonicalGrasp( 1,11))
//(CanonicalGrasp( 2,12))
//(CanonicalGrasp( 2,13))
//(CanonicalGrasp( 2,14))
//(CanonicalGrasp( 1,15))
//(CanonicalGrasp( 3,16))
//(CanonicalGrasp( 1,17))
//(CanonicalGrasp( 1,18))
//(CanonicalGrasp( 1,19))
//(CanonicalGrasp( 2,20))
//(CanonicalGrasp( 3,21))
//(CanonicalGrasp( 2,22))
//(CanonicalGrasp( 3,23))
//(CanonicalGrasp( 2,24))
//(CanonicalGrasp( 3,25))
//(CanonicalGrasp( 1,26))
//(CanonicalGrasp( 2,27))
//(CanonicalGrasp( 1,28))
//(CanonicalGrasp( 3,29))
//(CanonicalGrasp( 1,30))
//(CanonicalGrasp( 1,31))
//(CanonicalGrasp( 3,32))
//(CanonicalGrasp( 2,33));

//CanonicalGrasp::CanonicalGrasp(int _type, int _number):  GraspType(_type),  number(_number)
//{
//	// The infamous exceptions 16 20 21 23 25
//
//	// Hand orientation offset
//	double offset_orientation_angle=0.0; // angle in radians (30 degrees)
//	Eigen::Vector3d offset_orientation_vector=Eigen::Vector3d::UnitZ();
//
//	// Hand position offset
//	Eigen::Vector3d offset_position_vector(0,0,0);
//
//	// Hand canonical grasp orientation
//	Eigen::Matrix3d rotation_matrix;
//	rotation_matrix <<  x_g,  y_g,  z_g;
//
//	if(number==1)
//	{
//		grasp_name="Large Diameter";
//	}
//	else if(number==2)
//	{
//		grasp_name="Small Diameter";
//	}
//	else if(number==3)
//	{
//		grasp_name="Medium Wrap";
//	}
//	else if(number==4)
//	{
//		grasp_name="Adducted Thumb";
//	}
//	else if(number==5)
//	{
//		grasp_name="Light Tool";
//	}
//	else if(number==6)
//	{
//		grasp_name="Primastic 4 Finger";
//	}
//	else if(number==7)
//	{
//		grasp_name="Primastic 3 Finger";
//	}
//	else if(number==8)
//	{
//		grasp_name="Primastic 2 Finger";
//	}
//	else if(number==9)
//	{
//		offset_orientation_angle=0.52; // 30 degrees
//		offset_position_vector=Eigen::Vector3d(-0.066,0.0,-0.033);
//		grasp_name="Palmar Pinch";
//	}
//	else if(number==10)
//	{
//		grasp_name="Power Disk";
//	}
//	else if(number==11)
//	{
//		grasp_name="Power Sphere";
//	}
//	else if(number==12)
//	{
//		grasp_name="Precision Disk";
//	}
//	else if(number==13)
//	{
//		grasp_name="Precision Sphere";
//	}
//	else if(number==14)
//	{
//		grasp_name="Tripod";
//	}
//	else if(number==15)
//	{
//		grasp_name="Fixed Hook";
//	}
//	else if(number==16)
//	{
//		grasp_name="Lateral";
//	}
//	else if(number==17)
//	{
//		grasp_name="Index Finger Extension";
//	}
//	else if(number==18)
//	{
//		grasp_name="Extension Type";
//	}
//	else if(number==19)
//	{
//		grasp_name="Distal Type";
//	}
//	else if(number==20)
//	{
//		grasp_name="Writing Tripod";
//	}
//	else if(number==21)
//	{
//		grasp_name="Tripod Variation";
//	}
//	else if(number==22)
//	{
//		grasp_name="Parallel Extension";
//	}
//	else if(number==23)
//	{
//		grasp_name="Adduction Grip";
//	}
//	else if(number==24)
//	{
//		grasp_name="Tip Pinch";
//	}
//	else if(number==25)
//	{
//		grasp_name="Lateral Tripod";
//	}
//	else if(number==26)
//	{
//		grasp_name="Sphere 4 Finger";
//	}
//	else if(number==27)
//	{
//		grasp_name="Quadpod";
//	}
//	else if(number==28)
//	{
//		grasp_name="Sphere 3 Finger";
//	}
//	else if(number==29)
//	{
//		grasp_name="Stick";
//	}
//	else if(number==30)
//	{
//		grasp_name="Palmar";
//	}
//	else if(number==31)
//	{
//		grasp_name="Ring";
//	}
//	else if(number==32)
//	{
//		grasp_name="Ventral";
//	}
//	else if(number==33)
//	{
//		grasp_name="Inferior Pincer";
//	}
//
//	offset_rotation=Eigen::AngleAxisd(offset_orientation_angle,offset_orientation_vector);
//	offset_translation=Eigen::Translation3d(offset_position_vector);
//	orientation=Eigen::AngleAxisd(rotation_matrix);
//}



/////////////////////////////////////////
// GENERATE PRE HAND AND HAND POSTURES //
/////////////////////////////////////////
//CanonicalGrasp::CanonicalGrasp(int _type, int _number):  GraspType(_type),  number(_number)
//{
//// We assume pre-hand posture is always the same (open hand)
//hand_pre_posture.joint_states = boost::assign::list_of
//		(JointState("FFJ4",-10.0, 0, 0))
//		(JointState("FFJ3",  0.4, 0, 0))
//		(JointState("FFJ0",  0.4, 0, 0))
//		(JointState("MFJ4",  0.0, 0,0))
//		(JointState("MFJ3",  0.4, 0,0))
//		(JointState("MFJ0",  0.4, 0,0))
//		(JointState("RFJ4", -5.0, 0,0))
//		(JointState("RFJ3",  0.4, 0,0))
//		(JointState("RFJ0",  0.4, 0,0))
//		(JointState("LFJ5",  0.5, 0,0))
//		(JointState("LFJ4",-15.0, 0,0))
//		(JointState("LFJ3",  0.4, 0,0))
//		(JointState("LFJ0",  0.4, 0,0))
//		(JointState("THJ5",  0.0, 0,0))
//		(JointState("THJ4",  0.0, 0,0))
//		(JointState("THJ3",  0.0, 0,0))
//		(JointState("THJ2",  0.0, 0,0))
//		(JointState("THJ1",  0.0, 0,0));
//// only the known ones...
//if(number==9) // Palmar pinch
//{
//	//hand_joints: [FFJ4,FFJ3,FFJ0,MFJ4,MFJ3,MFJ0,RFJ4,RFJ3,RFJ0,LFJ5,LFJ4,LFJ3,LFJ0,THJ5,THJ4,THJ3,THJ2,THJ1]
//	hand_posture.joint_states = boost::assign::list_of
//		(JointState("FFJ4", -1.4775, 0, 0))
//		(JointState("FFJ3", 60.3900, 0, 0))
//		(JointState("FFJ0",  9.3600, 0, 0))
//		(JointState("MFJ4", -1.0200, 0,0))
//		(JointState("MFJ3", 25.4925, 0,0))
//		(JointState("MFJ0", 19.0750, 0,0))
//		(JointState("RFJ4", -3.0050,0,0))
//		(JointState("RFJ3", 17.6200,0,0))
//		(JointState("RFJ0", 12.9225,0,0))
//		(JointState("LFJ5",  0.0000,0,0))
//		(JointState("LFJ4", -6.5000,0,0))
//		(JointState("LFJ3", 12.0350,0,0))
//		(JointState("LFJ0", 23.8650,0,0))
//		(JointState("THJ5", 31.7375,0,0))
//		(JointState("THJ4", 58.7725,0,0))
//		(JointState("THJ3",  0.9225,0,0))
//		(JointState("THJ2",  6.8450,0,0))
//		(JointState("THJ1",  8.3425,0,0));
//	//hand_pre_posture.;
//}
//else if(number==14) // Tripod
//{
//	hand_posture.joint_states = boost::assign::list_of
//		(JointState("FFJ4", -1.4550, 0, 0))
//		(JointState("FFJ3", 37.7775, 0, 0))
//		(JointState("FFJ0", 72.9125, 0, 0))
//		(JointState("MFJ4", -1.2100, 0,0))
//		(JointState("MFJ3", 36.3675, 0,0))
//		(JointState("MFJ0", 85.6500, 0,0))
//		(JointState("RFJ4", -1.3200, 0,0))
//		(JointState("RFJ3", 29.6600, 0,0))
//		(JointState("RFJ0", 55.9650, 0,0))
//		(JointState("LFJ5",  0.0000, 0,0))
//		(JointState("LFJ4", -6.3000, 0,0))
//		(JointState("LFJ3", 20.0825, 0,0))
//		(JointState("LFJ0", 25.8100, 0,0))
//		(JointState("THJ5", 21.2350, 0,0))
//		(JointState("THJ4", 61.0250, 0,0))
//		(JointState("THJ3",  3.9075, 0,0))
//		(JointState("THJ2", 13.3225, 0,0))
//		(JointState("THJ1", 19.4350, 0,0));
//}
//else if(number==22) // Parallel extension
//{
//	hand_posture.joint_states = boost::assign::list_of
//		(JointState("FFJ4",  0.5200, 0, 0))
//		(JointState("FFJ3", 61.8125, 0, 0))
//		(JointState("FFJ0",  8.9175, 0, 0))
//		(JointState("MFJ4", -0.0400, 0,0))
//		(JointState("MFJ3", 61.4275, 0,0))
//		(JointState("MFJ0", 12.2600, 0,0))
//		(JointState("RFJ4",  0.2575, 0,0))
//		(JointState("RFJ3", 63.2050, 0,0))
//		(JointState("RFJ0",  8.6125, 0,0))
//		(JointState("LFJ5",  0.0000, 0,0))
//		(JointState("LFJ4", -0.1175, 0,0))
//		(JointState("LFJ3", 58.5550, 0,0))
//		(JointState("LFJ0", 20.5700, 0,0))
//		(JointState("THJ5", 31.7200, 0,0))
//		(JointState("THJ4", 62.6375, 0,0))
//		(JointState("THJ3",  1.7550, 0,0))
//		(JointState("THJ2",  7.2650, 0,0))
//		(JointState("THJ1",  9.0250, 0,0));
//}
//else if(number==24) // Tip pinch
//{
//	hand_posture.joint_states = boost::assign::list_of
//		(JointState("FFJ4",  0.2500, 0, 0))
//		(JointState("FFJ3", 25.8350, 0, 0))
//		(JointState("FFJ0", 93.1700, 0, 0))
//		(JointState("MFJ4", -1.0500, 0,0))
//		(JointState("MFJ3",  7.4675, 0,0))
//		(JointState("MFJ0", 18.9425, 0,0))
//		(JointState("RFJ4", -1.8775, 0,0))
//		(JointState("RFJ3",  5.5850, 0,0))
//		(JointState("RFJ0", 13.2425, 0,0))
//		(JointState("LFJ5",  0.0000, 0,0))
//		(JointState("LFJ4", -5.3000, 0,0))
//		(JointState("LFJ3",  4.2425, 0,0))
//		(JointState("LFJ0", 23.5950, 0,0))
//		(JointState("THJ5", 18.7050, 0,0))
//		(JointState("THJ4", 56.2925, 0,0))
//		(JointState("THJ3",  1.6375, 0,0))
//		(JointState("THJ2",  9.8200, 0,0))
//		(JointState("THJ1", 29.4300, 0,0));
//}
//else
//	hand_posture=hand_pre_posture; // DUMMY
//
//}
