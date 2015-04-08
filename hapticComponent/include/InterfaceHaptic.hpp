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

#include "commonHapticComponent.h"

class InterfaceHaptic{

public:

	/// Not default constructor instead use a virtual constructor
	virtual InterfaceHaptic* Create() = 0;
	virtual ~InterfaceHaptic(){};

	/// Initializes the component
	virtual void Init() = 0;

	/// calibrate the haptic device
	virtual bool calibrate() = 0;

	/// start the haptic device
	virtual bool start() = 0;

	/// stop the haptic device
	virtual bool stop() = 0;

	/// old Function, not used	
	virtual bool getPositionTX(mt::Transform &) = 0;

	/// get the position Xh,Yh,Zh on haptic coordinates
	/// @param[in,out]	mt::Transform it saves the 3 vector haptic position
	virtual bool getPosition(mt::Transform &) = 0;

	/// get the velocity of each joint
	/// @param[in,out] Vect6 contains the speeds of haptic Pose and torque joints
	virtual bool getVelocity(Vect6 &) = 0;

	/// set the force at the haptic pose and torque joints
	/// @param[in,out] const Vect6 it contains the forces to be set to the haptic
	virtual bool setForce(const Vect6) = 0;

	/// get the state of the stylus button
	virtual bool getButtom() = 0;

	/// get the position of the joints
	/// @param[in,out] Vect6 it contains the position of each joint of the haptic
	virtual bool getJointPosition(Vect6 &) = 0;

	/// set the torque to each torque
	/// @param[in,out] Vect6 it contains the torque to be set at each joint motor
	virtual bool setMotorTorque(const Vect6) = 0;

	/// get the jacobian 
	/// @param[in,out] ublas::matrix<mt::Scalar> Matrix that contains the Jacobian
	virtual bool getJacobian(ublas::matrix<mt::Scalar> &) = 0;

	/// get the jacobian Transpose 
	virtual bool getJacobianTranspose(ublas::matrix<mt::Scalar> &) = 0;

	/// set the Workspace Limits to a given cube size
	/// @param[in,out] minCubeLimits minimum size for the cube workspace
	/// @param[in,out] maxCubeLimits maximum size for the cune workspace
	virtual bool SetWorkSpaceLimits(mt::Vector3 minCubeLimits, mt::Vector3 maxCubeLimits) = 0;


};