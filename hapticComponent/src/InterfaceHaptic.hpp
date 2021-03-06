/* \brief

This is the Interface for the Phantom 1.5 6 d.o.f High Force Version
Haptic device

This is the Abstract Base Class that defines the interface for
this component

The API design uses the factory method software pattern
see : Chapter 3 Patterns of Book API C++ Design by M. Reedy 2011

some features are:

1. use of virtual constructors
2. use a factory method to enable the creation of the API class using a derived class
*/

/*
PROJECT:	3DPOINTER
COMPONENT:	HAPTIC
DATE:		26.03.15
AUTHOR:		HENRY PORTILLA
SPECIAL
THANKS:		GOD

*/
#pragma once
#include "commonHapticComponent.h"

class InterfaceHaptic{

public:
	
	/// Not default constructor instead use a virtual constructor
	virtual InterfaceHaptic* Create() = 0;
	virtual ~InterfaceHaptic(){};

	/// initializes the component
	virtual void Init(bool &init) = 0;

	/// start the connection with the haptic device
	virtual void startConnection() = 0;

	/// set the Workspace Limits to a given cube size
	/// @param[in,out] minCubeLimits minimum size for the cube workspace
	/// @param[in,out] maxCubeLimits maximum size for the cune workspace
	virtual bool setWorkSpaceLimits(mt::Vector3 minCubeLimits, mt::Vector3 maxCubeLimits) = 0;

	/// get the Workspace Limits to a given cube size
	/// @param[in,out] minCubeLimits minimum size for the cube workspace
	/// @param[in,out] maxCubeLimits maximum size for the cune workspace
	virtual void getWorkSpaceLimits(mt::Vector3 MinCubicLimits, mt::Vector3 MaxCubicLimits) = 0;

	/// get the translation and orientation on yaw,pitch, roll representation from the haptic
	/// @param[in,out] hapticPosition save the haptic position in original coordinates or device coordinates
	virtual void getHapticPosition(mt::Transform &hapticPosition) = 0;

	/// get the compensation force from the haptic device
	/// @param[in,out] forceVector it contains the force vector in position and torque for the haptic device
	virtual void getGravityCompensation(CompensationForce &forceVector) = 0;

	/// set the gravity compensation ON or OFF
	/// @param[in] gravityEnable boolean flag to indicate enabling or disabling the gravity compensation
	virtual void setGravityCompensation(bool &gravityEnable) = 0;

	/// set the force at the haptic pose and torque joints
	/// @param[in,out] const Vect6 it contains the forces to be set to the haptic
	virtual bool setForce(const Vect6) = 0;

	/// set the ka and Kd gains to stabilize gravity compensation
	virtual void setKa(double Ka) = 0;

	virtual void setKd(double kd) = 0;

	/// get the jacobian 
	/// @param[in,out] ublas::matrix<mt::Scalar> Matrix that contains the Jacobian
	virtual bool getJacobian(ublas::matrix<mt::Scalar> &Jacobian) = 0;

	/// get the jacobian Transpose 
	virtual bool getJacobianTranspose(ublas::matrix<mt::Scalar> &JacobianT) = 0;

	/// close the connection with the haptic device
	virtual void closeConnection() = 0;


};