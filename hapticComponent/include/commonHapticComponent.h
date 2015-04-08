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

//Shared struct for the haptic threads
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

	// workspace limits 
	HDfloat Xmin;
	HDfloat Xmax;
	HDfloat Ymin;
	HDfloat Ymax;
	HDfloat Zmin;
	HDfloat Zmax;

};