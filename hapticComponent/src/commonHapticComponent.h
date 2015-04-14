/*! \brief

This header contain common definitions used in the
HapticComponent

*/

#pragma once

#if defined(WIN32)
#define WIN32_LEAN_AND_MEAN
#define _WINSOCKAPI_
#include <windows.h>
#endif

//mt library
#include <mt/mt.h>

//HD headers from the Open Haptics
#include <HD/hd.h>
#include <HDU/hduVector.h>
//#include <HDU/hduError.h>

//uBLAS library
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace ublas = boost::numeric::ublas;

// typedef for the Six component Vector
typedef ublas::vector<mt::Scalar> Vect6;

/// haptic device ID
///< P1_5_6DOF		phantom 1.5 6 d.o.f
///< P1_5_6DOF_HF	phantom 1.5 6 d.o.f high Force
///< OMNI			omni 6 d.o.f, 3 are actuated, orientation not
enum hapticDevice {P1_5_6DOF = 0, P1_5_6DOF_HF = 1, OMNI = 2};

const double gravityK = 9.8;

/// some haptic parameters used for the gravity compensation
/// for details see the Thesis of Carlos Aldana pages 94,99
/// titled: "Consensus Control in Robot Networks and Cooperative
///			Teleoperation: An Operational Space Approach"
///			Technical University of Catalonia January 2015
struct hapticPhysicalParameters{

	// masses
	double m1;
	double m2;
	double m3;

	// lengths
	double l1;
	double l2;
	double l3;

	// center of masses
	double lc1;
	double lc2;
	double lc3;
};

hapticPhysicalParameters Premium_1_5_6DOF, Premium_1_5_6DOF_HF, Premium_Omni;

/// structure that saves the current joint position in angles
struct jointAngles{

	double q1;
	double q2;
	double q3;
	double q4;
	double q5;
	double q6;
};

/// structure that saves the current gravity compensation for the haptic device
struct CompensationForce{

	double f1;
	double f2;
	double f3;
	double f4;
	double f5;
	double f6;

};

///Shared struct for the haptic threads
struct HapticState{
	HapticState() : phForce(0, 0, 0), phTorqe(0, 0, 0){};

	HDint phButton;
	HDdouble phPos[16];
	hduVector3Dd phVel;
	HDdouble phAngVel[3];
	hduVector3Dd phForce;
	hduVector3Dd phTorqe;
	HDdouble phBaseJoints[3];
	HDdouble phGimbalJoints[3];
	HDlong phMotorTorque[6];
	hduVector3Dd phGravityForce;

	// workspace limits 
	HDfloat Xmin;
	HDfloat Xmax;
	HDfloat Ymin;
	HDfloat Ymax;
	HDfloat Zmin;
	HDfloat Zmax;

};