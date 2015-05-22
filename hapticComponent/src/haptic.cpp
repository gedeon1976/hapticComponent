/***************************************************************************
*   Copyright (C) 2007 by Emmanuel Nuño                                   *
*   emmanuel.nuno@upc.edu
*
*	 Modified: 2012, 2015 by Henry Portilla
*
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#include "haptic.h"

////////////////////////////////////////////////////////////////////////////
// interface realization (implementation) for the API of this component
////////////////////////////////////////////////////////////////////////////



void Haptic::Init(bool &init){

	m_phState = new HapticState;

	m_phState->phMotorTorque[0] = 0;
	m_phState->phMotorTorque[1] = 0;
	m_phState->phMotorTorque[2] = 0;
	m_phState->phMotorTorque[3] = 0;
	m_phState->phMotorTorque[4] = 0;
	m_phState->phMotorTorque[5] = 0;

	m_hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(m_error = hdGetError()))
	{
		std::cout << "Failed to initialize haptic device" << std::endl;
		init = false;
		m_init = init;
		return;
	}

	init = true;
	m_init = init;
	std::cout << "Found Haptic: " << hdGetString(HD_DEVICE_MODEL_TYPE) << std::endl;

	hdEnable(HD_FORCE_OUTPUT);

	// load parameters for gravity compensation

	////////////////////////////////////////////
	// Premium 1.5 6 DOF 
	// masses
	m_phState->Premium_1_5_6DOF.m1 = 0.026;
	m_phState->Premium_1_5_6DOF.m2 = 0.01;
	m_phState->Premium_1_5_6DOF.m3 = 0.09;

	// lengths
	m_phState->Premium_1_5_6DOF.l1 = 0.21;
	m_phState->Premium_1_5_6DOF.l2 = 0.21;
	m_phState->Premium_1_5_6DOF.l3 = 0.07;

	// center of masses
	m_phState->Premium_1_5_6DOF.lc1 = 0.105;
	m_phState->Premium_1_5_6DOF.lc2 = 0.21;
	m_phState->Premium_1_5_6DOF.lc3 = 0.07;

	////////////////////////////////////////////
	// Premium 1.5 6 DOF High Force
	// masses
	m_phState->Premium_1_5_6DOF_HF.m1 = 0.0056;
	m_phState->Premium_1_5_6DOF_HF.m2 = 0.005;
	m_phState->Premium_1_5_6DOF_HF.m3 = 0.09;

	// lengths
	m_phState->Premium_1_5_6DOF_HF.l1 = 0.21;
	m_phState->Premium_1_5_6DOF_HF.l2 = 0.21;
	m_phState->Premium_1_5_6DOF_HF.l3 = 0.1;

	// center of masses
	m_phState->Premium_1_5_6DOF_HF.lc1 = 0.105;
	m_phState->Premium_1_5_6DOF_HF.lc2 = 0.21;
	m_phState->Premium_1_5_6DOF_HF.lc3 = 0.1;

	////////////////////////////////////////////
	// Omni 6 DOF 
	// masses
	m_phState->Premium_Omni.m1 = 0.0;
	m_phState->Premium_Omni.m2 = 0.02;
	m_phState->Premium_Omni.m3 = 0.055;

	// lengths
	m_phState->Premium_Omni.l1 = 0.16;
	m_phState->Premium_Omni.l2 = 0.19;
	m_phState->Premium_Omni.l3 = 0.0;

	// center of masses
	m_phState->Premium_Omni.lc1 = 0.0;
	m_phState->Premium_Omni.lc2 = 0.0;
	m_phState->Premium_Omni.lc3 = 0.0;
	
}

// start the connection with the haptic device
void Haptic::startConnection(){

	Vect6 ForceonHaptic(6);
	
	calibrate();
	start();

	// set initial values to the force to be send

	ForceonHaptic[0] = 0;
	ForceonHaptic[1] = 0;
	ForceonHaptic[2] = 0;
	ForceonHaptic[3] = 0;
	ForceonHaptic[4] = 0;
	ForceonHaptic[5] = 0;

	setForce(ForceonHaptic);

}

// set the workspace for this haptic device, usually a cube
bool Haptic::setWorkSpaceLimits(mt::Vector3 minCubeLimits, mt::Vector3 maxCubeLimits){

	if (m_init == false) return false;

	m_phState->Xmin = minCubeLimits[0];	m_phState->Xmax = maxCubeLimits[0];
	m_phState->Ymin = minCubeLimits[1];	m_phState->Ymax = maxCubeLimits[1];
	m_phState->Zmin = minCubeLimits[2];	m_phState->Zmax = maxCubeLimits[2];

	return true;
}

// get the current workspace area for this haptic device, if not set indicate this
void Haptic::getWorkSpaceLimits(mt::Vector3 minCubeLimits, mt::Vector3 maxCubeLimits){

}

// get the Haptic position: pose and orientation
void Haptic::getHapticPosition(mt::Transform &hapticPosition){

	getPosition(hapticPosition);

}

// get the current force applied to the gravity compensation
void Haptic::getGravityCompensation(CompensationForce &forceVector){

	forceVector.f1 = m_phState->phGravityForce[0];
	forceVector.f2 = m_phState->phGravityForce[1];
	forceVector.f3 = m_phState->phGravityForce[2];

	forceVector.f4 = m_phState->phGravityTorqe[0];
	forceVector.f5 = m_phState->phGravityTorqe[1];
	forceVector.f6 = m_phState->phGravityTorqe[2];

}

// set the gravity compensation for this haptic device
void Haptic::setGravityCompensation(bool &gravityEnable){

	// check gravity check is ON or OFF
	if (gravityEnable){

		m_phState->gravityEnable = 1;
			
	}
	else{
	
		m_phState->gravityEnable = 0;
		
	}

}

// set the haptic force
bool Haptic::setForce(const Vect6 force)
{
	if (m_init == false) return false;

	m_phState->phForce[0] = mt::getValue(force[0]);
	m_phState->phForce[1] = mt::getValue(force[1]);
	m_phState->phForce[2] = mt::getValue(force[2]);

	m_phState->phTorqe[0] = mt::getValue(force[3]);
	m_phState->phTorqe[1] = mt::getValue(force[4]);
	m_phState->phTorqe[2] = mt::getValue(force[5]);

	return true;
}

// get the Jacobian for the haptic
bool Haptic::getJacobian(ublas::matrix<mt::Scalar> &j)
{
	Vect6 q(6);
	getJointPosition(q);

	const mt::Scalar l1 = 0.21;
	const mt::Scalar l2 = 0.21;

	const mt::Scalar s1 = sin(q[0]);
	const mt::Scalar s2 = sin(q[1]);
	const mt::Scalar s3 = sin(q[2]);
	const mt::Scalar s4 = sin(q[3]);
	const mt::Scalar s5 = sin(q[4]);


	const mt::Scalar c1 = cos(q[0]);
	const mt::Scalar c2 = cos(q[1]);
	const mt::Scalar c3 = cos(q[2]);
	const mt::Scalar c4 = cos(q[3]);
	const mt::Scalar c5 = cos(q[4]);

	j.resize(6, 6);

	j(0, 0) = c1*(l1*c1 + l2*s3);
	j(1, 0) = 0.0;
	j(2, 0) = -s1*(l1*c2 + l2*s3);

	j(0, 1) = s1*(-l1*s2 + l2*c3);
	j(1, 1) = l1*c2 + l2*s3;
	j(2, 1) = c1*(-l1*s2 + l2*c3);

	j(0, 2) = l2*s1*c3;
	j(1, 2) = l2*s3;
	j(2, 2) = l2*c1*c3;

	j(0, 3) = 0.0;
	j(1, 3) = 0.0;
	j(2, 3) = 0.0;

	j(0, 4) = 0.0;
	j(1, 4) = 0.0;
	j(2, 4) = 0.0;

	j(0, 5) = 0.0;
	j(1, 5) = 0.0;
	j(2, 5) = 0.0;

	j(3, 0) = 0.0;
	j(4, 0) = 1.0;
	j(5, 0) = 0.0;

	j(3, 1) = -c1;
	j(4, 1) = 0.0;
	j(5, 1) = s1;

	j(3, 2) = -c1;
	j(4, 2) = 0.0;
	j(5, 2) = s1;

	j(3, 3) = s1*s3;
	j(4, 3) = -c3;
	j(5, 3) = c1*s3;

	j(3, 4) = -s1*c3*s4 - c1*c4;
	j(4, 4) = -s3*s4;
	j(5, 4) = -c1*c3*s4 + s1*c4;

	j(3, 5) = s1*c3*c4*c5 - c1*s4*c5 - s1*s3*s5;
	j(4, 5) = s3*c4*c5 + c3*s5;
	j(5, 5) = c1*c3*c4*c5 + s1*s4*c5 - c1*s3*s5;

	return true;
}

// get the Jacobian transposed
bool Haptic::getJacobianTranspose(ublas::matrix<mt::Scalar> &j)
{
	Vect6 q(6);
	getJointPosition(q);

	const mt::Scalar l1 = 0.21;
	const mt::Scalar l2 = 0.21;

	const mt::Scalar s1 = sin(q[0]);
	const mt::Scalar s2 = sin(q[1]);
	const mt::Scalar s3 = sin(q[2]);
	const mt::Scalar s4 = sin(q[3]);
	const mt::Scalar s5 = sin(q[4]);


	const mt::Scalar c1 = cos(q[0]);
	const mt::Scalar c2 = cos(q[1]);
	const mt::Scalar c3 = cos(q[2]);
	const mt::Scalar c4 = cos(q[3]);
	const mt::Scalar c5 = cos(q[4]);

	j.resize(6, 6);

	j(0, 0) = c1*(l1*c1 + l2*s3);
	j(0, 1) = 0.0;
	j(0, 2) = -s1*(l1*c2 + l2*s3);

	j(1, 0) = s1*(-l1*s2 + l2*c3);
	j(1, 1) = l1*c2 + l2*s3;
	j(1, 2) = c1*(-l1*s2 + l2*c3);

	j(2, 0) = l2*s1*c3;
	j(2, 1) = l2*s3;
	j(2, 2) = l2*c1*c3;

	j(3, 0) = 0.0;
	j(3, 1) = 0.0;
	j(3, 2) = 0.0;

	j(4, 0) = 0.0;
	j(4, 1) = 0.0;
	j(4, 2) = 0.0;

	j(5, 0) = 0.0;
	j(5, 1) = 0.0;
	j(5, 2) = 0.0;

	j(0, 3) = 0.0;
	j(0, 4) = 1.0;
	j(0, 5) = 0.0;

	j(1, 3) = -c1;
	j(1, 4) = 0.0;
	j(1, 5) = s1;

	j(2, 3) = -c1;
	j(2, 4) = 0.0;
	j(2, 5) = s1;

	j(3, 3) = s1*s3;
	j(3, 4) = -c3;
	j(3, 5) = c1*s3;

	j(4, 3) = -s1*c3*s4 - c1*c4;
	j(4, 4) = -s3*s4;
	j(4, 5) = -c1*c3*s4 + s1*c4;

	j(5, 3) = s1*c3*c4*c5 - c1*s4*c5 - s1*s3*s5;
	j(5, 4) = s3*c4*c5 + c3*s5;
	j(5, 5) = c1*c3*c4*c5 + s1*s4*c5 - c1*s3*s5;

	return true;
}

// close the connection with the device
void Haptic::closeConnection(){

	// stop the haptic device
	stop();

}


////////////////////////////////////////////////////////////////////////////
// internal methods implementation
////////////////////////////////////////////////////////////////////////////

void Haptic::getHapticDevice(int &hapticType){}

bool Haptic::calibrate(){

	if (m_init == false) return false;

	int supportedCalibrationStyles;
	int calibrationStyle;

	std::cout << "Starting Haptic Calibration" << std::endl;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
	{
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
	{
		calibrationStyle = HD_CALIBRATION_INKWELL;
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
	{
		calibrationStyle = HD_CALIBRATION_AUTO;
	}

	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
	{
		std::cout << "Please prepare for manual calibration." << std::endl;
		std::cout << "Place the Haptic at reset position and press Enter" << std::endl;

		//std::cin.get();

		hdUpdateCalibration(calibrationStyle);
		if (hdCheckCalibration() == HD_CALIBRATION_OK)
		{
			std::cout << "Calibration complete" << std::endl;
		}
		if (HD_DEVICE_ERROR(m_error = hdGetError()))
		{
			std::cout << "Reset encoders reset failed." << std::endl;
			return false;
		}
	}
	return true;
}

bool Haptic::start(){

	if (m_init == false) return false;

	hdScheduleAsynchronous(hdState, (void *)m_phState, HD_MAX_SCHEDULER_PRIORITY);
	//hdScheduleAsynchronous(FrictionlessPlaneCallback, (void *)m_phState, HD_MAX_SCHEDULER_PRIORITY);

	hdStartScheduler();

	if (HD_DEVICE_ERROR(m_error = hdGetError()))
	{
		std::cout << "Failed to start the scheduler" << std::endl;
		return false;
	}

	std::cout << "Scheduler Started" << std::endl;

#ifdef WIN32
	Sleep(15);
#elif
	sleep(15);
#endif
	return true;
}

bool Haptic::stop(){

	if (m_init == false) return false;

	hdStopScheduler();
	hdDisableDevice(m_hHD);

	std::cout << "Haptic has been Stoped" << std::endl;
	return true;
}

void Haptic::setGravityVector(jointAngles &currentAngles, CompensationForce &currentForce){

	

}


bool Haptic::getPositionTX(mt::Transform &trans)
{
	if (m_init == false) return false;

	mt::Matrix3x3 rotM(m_phState->phPos[0], m_phState->phPos[4], m_phState->phPos[8],
		m_phState->phPos[1], m_phState->phPos[5], m_phState->phPos[9],
		m_phState->phPos[2], m_phState->phPos[6], m_phState->phPos[10]);

	//rotM.setIdentity();
	mt::Matrix3x3 rz(0, 1, 0,
		-1, 0, 0,
		0, 0, 1);

	mt::Matrix3x3 ry(0, 0, -1,
		0, 1, 0,
		1, 0, 0);


	mt::Matrix3x3 compose(0, 0, 1,
		1, 0, 0,
		0, 1, 0);
	rotM = rotM*rz;
	rotM = rotM*ry;

	rotM = compose*rotM;

	const mt::Rotation phRot(rotM);
	const mt::Vector3  T(m_phState->phPos[14], m_phState->phPos[12], m_phState->phPos[13]);
	//const mt::Vector3  T(0.0,0.0,m_phState->phPos[13]);
	trans.setRotation(phRot);
	trans.setTranslation(T);

	return true;
}

bool Haptic::getPosition(mt::Transform &trans)
{
	if (m_init == false) return false;

	mt::Matrix3x3 rotM(m_phState->phPos[0], m_phState->phPos[4], m_phState->phPos[8],
		m_phState->phPos[1], m_phState->phPos[5], m_phState->phPos[9],
		m_phState->phPos[2], m_phState->phPos[6], m_phState->phPos[10]);

	const mt::Rotation phRot(rotM);
	const mt::Vector3  T(m_phState->phPos[12], m_phState->phPos[13], m_phState->phPos[14]);
	trans.setRotation(phRot);
	trans.setTranslation(T);

	return true;
}

bool Haptic::getJointPosition(Vect6 &phJoints)
{
	if (m_init == false) return false;

	phJoints[0] = m_phState->phBaseJoints[0];
	phJoints[1] = m_phState->phBaseJoints[1];
	phJoints[2] = m_phState->phBaseJoints[2];

	phJoints[3] = m_phState->phGimbalJoints[0];
	phJoints[4] = m_phState->phGimbalJoints[1];
	phJoints[5] = m_phState->phGimbalJoints[2];

	return true;
}

bool Haptic::getJointPosition(jointAngles &currentAngles)
{
	if (m_init == false) return false;

	currentAngles.q1 = m_phState->phBaseJoints[0];
	currentAngles.q2 = m_phState->phBaseJoints[1];
	currentAngles.q3 = m_phState->phBaseJoints[2];

	currentAngles.q4 = m_phState->phGimbalJoints[0];
	currentAngles.q5 = m_phState->phGimbalJoints[1];
	currentAngles.q6 = m_phState->phGimbalJoints[2];

	return true;
}

bool Haptic::getVelocity(Vect6 &phVel)
{
	if (m_init == false) return false;

	phVel[0] = m_phState->phVel[2];
	phVel[1] = m_phState->phVel[0];
	phVel[2] = m_phState->phVel[1];
	phVel[3] = m_phState->phAngVel[0];
	phVel[4] = m_phState->phAngVel[1];
	phVel[5] = m_phState->phAngVel[2];

	return true;
}

bool Haptic::getButton()
{
	if (m_init == false) return false;

	if (m_phState->phButton == 1)	return true;

	return false;
}



bool Haptic::setMotorTorque(const Vect6 motorTorque)
{
	if (m_init == false) return false;

	m_phState->phMotorTorque[0] = mt::getValue(motorTorque[0]);
	m_phState->phMotorTorque[1] = mt::getValue(motorTorque[1]);
	m_phState->phMotorTorque[2] = mt::getValue(motorTorque[2]);
	m_phState->phMotorTorque[3] = mt::getValue(motorTorque[3]);
	m_phState->phMotorTorque[4] = mt::getValue(motorTorque[4]);
	m_phState->phMotorTorque[5] = mt::getValue(motorTorque[5]);

	return true;
}





HDCallbackCode HDCALLBACK hdState(void *pState)
{
	HapticState *phState = (HapticState *)pState;
	// Stiffnes, i.e. k value, of the plane.  Higher stiffness results
	// in a harder surface.
	const double planeStiffness = 1;
	// Amount of force the user needs to apply in order to pop through
	// the plane.
	const double popthroughForceThreshold = 25.0;

	// Plane direction changes whenever the user applies sufficient
	// force to popthrough it.
	// 1 means the plane is facing +Y.
	// -1 means the plane is facing -Y.
	static int directionFlagX = 1, directionFlagX_bottom = 1, directionFlagX_top = -1;
	static int directionFlagY = 1, directionFlagY_bottom = 1, directionFlagY_top = -1;
	static int directionFlagZ = 1, directionFlagZ_bottom = 1, directionFlagZ_top = -1;
	float X_limit = 0, Y_limit = 0, Z_limit = 0;

	// start haptic frame
	hdBeginFrame(hdGetCurrentDevice());
	hdGetIntegerv(HD_CURRENT_BUTTONS, &phState->phButton);
	hdGetDoublev(HD_CURRENT_TRANSFORM, phState->phPos);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, phState->phBaseJoints);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, phState->phGimbalJoints);
	hdGetDoublev(HD_CURRENT_VELOCITY, phState->phVel);
	hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, phState->phAngVel);
	hdSetDoublev(HD_CURRENT_FORCE, phState->phForce);
	hdSetDoublev(HD_CURRENT_TORQUE, phState->phTorqe);
	//hdSetLongv(HD_CURRENT_MOTOR_DAC_VALUES, phState->phMotorTorque);

	// ADD CUBIC WORKSPACE LIMITS
	// Get the position of the device.
	hduVector3Dd position;
	hdGetDoublev(HD_CURRENT_POSITION, position);

	// If the user has penetrated the plane, set the device force to 
	// repel the user in the direction of the surface normal of the plane.
	// Penetration occurs if the plane is facing in + and the user's position
	// is negative, or vice versa.
	double penetrationDistance = 0;
	double k = planeStiffness;
	hduVector3Dd forceDirection;
	hduVector3Dd Delta_x, Delta_y, Delta_z;
	hduVector3Dd F, Fx, Fy, Fz;
	// Limits on X
	if ((position[0] <= phState->Xmin && directionFlagX_bottom > 0) ||
		(position[0] >= phState->Xmax) && (directionFlagX_top < 0))
	{
		// Create a force vector repelling the user from the plane proportional
		// to the penetration distance, using F=kDelta_X where k is the plane 
		// stiffness and Delta_X is the penetration vector.  Since the plane is 
		// oriented at the X=X_limit, the force direction is always either directly 
		// outward or inward, i.e. either (1,0,0) or (-1,0,0).
		if (position[0] <= phState->Xmin){
			directionFlagX = 1;
			X_limit = phState->Xmin;
		}
		else if (position[0] >= phState->Xmax){
			directionFlagX = -1;
			X_limit = phState->Xmax;
		}
		penetrationDistance = fabs(position[0] - X_limit);
		forceDirection.set(directionFlagX, 0, 0);

		// Hooke's law explicitly:			
		Delta_x = penetrationDistance*forceDirection;
		Fx = k*Delta_x;

		// If the user applies sufficient force, pop through the plane
		// by reversing its direction.  Otherwise, apply the repel
		// force.
		if (Fx.magnitude() > popthroughForceThreshold)
		{
			Fx.set(0.0, 0.0, 0.0);
		}

	}
	// Limits on Y
	if ((position[1] <= phState->Ymin && directionFlagY_bottom > 0) ||
		(position[1] >= phState->Ymax) && (directionFlagY_top < 0))
	{
		// Create a force vector repelling the user from the plane proportional
		// to the penetration distance, using F=kDelta_Y where k is the plane 
		// stiffness and Delta_Y is the penetration vector.  Since the plane is 
		// oriented at the Y=Y_limit, the force direction is always either directly 
		// upward or downward, i.e. either (0,1,0) or (0,-1,0).
		if (position[1] <= phState->Ymin){
			directionFlagY = 1;
			Y_limit = phState->Ymin;
		}
		else if (position[1] >= phState->Ymax){
			directionFlagY = -1;
			Y_limit = phState->Ymax;
		}
		penetrationDistance = fabs(position[1] - Y_limit);
		forceDirection.set(0, directionFlagY, 0);

		// Hooke's law explicitly:			
		Delta_y = penetrationDistance*forceDirection;
		Fy = k*Delta_y;

		// If the user applies sufficient force, pop through the plane
		// by reversing its direction.  Otherwise, apply the repel
		// force.
		if (Fy.magnitude() > popthroughForceThreshold)
		{
			Fy.set(0.0, 0.0, 0.0);
		}

	}
	// Limits on Z
	if ((position[2] <= phState->Zmin && directionFlagZ_bottom > 0) ||
		(position[2] >= phState->Zmax) && (directionFlagZ_top < 0))
	{
		// Create a force vector repelling the user from the plane proportional
		// to the penetration distance, using F=kDelta_Z where k is the plane 
		// stiffness and Delta_Z is the penetration vector.  Since the plane is 
		// oriented at the Z=Z_limit, the force direction is always either directly 
		// outward or inward, i.e. either (0,0,1) or (0,0,-1).
		if (position[2] <= phState->Zmin){
			directionFlagZ = 1;
			Z_limit = phState->Zmin;
		}
		else if (position[2] >= phState->Zmax){
			directionFlagZ = -1;
			Z_limit = phState->Zmax;
		}
		penetrationDistance = fabs(position[2] - Z_limit);
		forceDirection.set(0, 0, directionFlagZ);

		// Hooke's law explicitly:			
		Delta_z = penetrationDistance*forceDirection;
		Fz = k*Delta_z;

		// If the user applies sufficient force, pop through the plane
		// by reversing its direction.  Otherwise, apply the repel
		// force.
		if (Fz.magnitude() > popthroughForceThreshold)
		{
			Fz.set(0.0, 0.0, 0.0);
		}

	}
	// Final Force Sum F = (Fx + Fy + Fz)
	// It produces compelling planes with K = 1
	F = Fx + Fy + Fz;
	hdSetDoublev(HD_CURRENT_FORCE, F);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Add the gravity compensation force vector
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	HDdouble gravityForceVector[6];
	jointAngles currentAngles;
	double value1 = 0, value2 = 0, value3 = 0;
	int hapticDeviceID= 1000;

	// get the current joint positions
	currentAngles.q1 = phState->phBaseJoints[0];
	currentAngles.q2 = phState->phBaseJoints[1];
	currentAngles.q3 = phState->phBaseJoints[2];

	currentAngles.q4 = phState->phGimbalJoints[0];
	currentAngles.q5 = phState->phGimbalJoints[1];
	currentAngles.q6 = phState->phGimbalJoints[2];

	// select the haptic device automatically
	std::string hapticType(hdGetString(HD_DEVICE_MODEL_TYPE));

	if (hapticType.compare("PHANTOM Premium 1.5") == 0){

		hapticDeviceID = 0;
	}

	if (hapticType.compare("PHANTOM Premium 1.5 6DOF") == 0){

		hapticDeviceID = 1;
	}

	if (hapticType.compare("PHANTOM Omni") == 0){

		hapticDeviceID = 2;
	}

	///////////////////////////////////////////////////////////////////////////
	// GRAVITY COMPENSATION
	// check if gravity compensation is enabled or not
	if (phState->gravityEnable){
	
		switch (hapticDeviceID)
		{

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//   Premium 1.5 6 D.O.F
		case P1_5_6DOF:

			value1 = phState->Premium_1_5_6DOF.m2*phState->Premium_1_5_6DOF.lc2 + phState->Premium_1_5_6DOF.m3*phState->Premium_1_5_6DOF.l2;
			value2 = phState->Premium_1_5_6DOF.m3*phState->Premium_1_5_6DOF.l3;
			value3 = phState->Premium_1_5_6DOF.m3*phState->Premium_1_5_6DOF.lc3;

			gravityForceVector[0] = 0;
			gravityForceVector[1] = (phState->Premium_1_5_6DOF.m1*phState->Premium_1_5_6DOF.lc1 + phState->Premium_1_5_6DOF.m2*phState->Premium_1_5_6DOF.l1
				+ phState->Premium_1_5_6DOF.m3*phState->Premium_1_5_6DOF.l1)*cos(currentAngles.q2)
				+ value1*sin(currentAngles.q3) + value2*cos(currentAngles.q3 + currentAngles.q5);
			gravityForceVector[2] = value1*sin(currentAngles.q3) + value3*cos(currentAngles.q3 + currentAngles.q5);
			gravityForceVector[3] = value3*sin(currentAngles.q3 + currentAngles.q5)*sin(currentAngles.q4);
			gravityForceVector[4] = value3*cos(currentAngles.q3)*cos(currentAngles.q5);
			gravityForceVector[5] = 0;

		break;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//   Premium 1.5 6 D.O.F
		case P1_5_6DOF_HF:

			value1 = phState->Premium_1_5_6DOF_HF.m2*phState->Premium_1_5_6DOF_HF.lc2 + phState->Premium_1_5_6DOF_HF.m3*phState->Premium_1_5_6DOF_HF.l2;
			value2 = phState->Premium_1_5_6DOF_HF.m3*phState->Premium_1_5_6DOF_HF.l3;
			value3 = phState->Premium_1_5_6DOF_HF.m3*phState->Premium_1_5_6DOF_HF.lc3;

			gravityForceVector[0] = 0;
			gravityForceVector[1] = (phState->Premium_1_5_6DOF_HF.m1*phState->Premium_1_5_6DOF_HF.lc1 + phState->Premium_1_5_6DOF_HF.m2*phState->Premium_1_5_6DOF_HF.l1
				+ phState->Premium_1_5_6DOF_HF.m3*phState->Premium_1_5_6DOF_HF.l1)*cos(currentAngles.q2)
				+ value1*sin(currentAngles.q3) + value2*cos(currentAngles.q3 + currentAngles.q5);
			gravityForceVector[2] = value1*sin(currentAngles.q3) + value3*cos(currentAngles.q3 + currentAngles.q5);
			gravityForceVector[3] = value3*sin(currentAngles.q3 + currentAngles.q5)*sin(currentAngles.q4);
			gravityForceVector[4] = value3*cos(currentAngles.q3)*cos(currentAngles.q5);
			gravityForceVector[5] = 0;

		break;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// OMNI
		case OMNI:

			value1 = phState->Premium_Omni.m3*phState->Premium_Omni.l2;
			value2 = phState->Premium_Omni.m2*phState->Premium_Omni.l1;

			gravityForceVector[0] = 0;
			gravityForceVector[1] = value1*sin(currentAngles.q2 + currentAngles.q3)
				+ (value1 + value2)*cos(currentAngles.q2);
			gravityForceVector[2] = value1*sin(currentAngles.q2 + currentAngles.q3);
			gravityForceVector[3] = 0;
			gravityForceVector[4] = 0;
			gravityForceVector[5] = 0;

			break;

		default:	// No haptic device or not well detected

			gravityForceVector[0] = 0;
			gravityForceVector[1] = 0;
			gravityForceVector[2] = 0;
			gravityForceVector[3] = 0;
			gravityForceVector[4] = 0;
			gravityForceVector[5] = 0;
			break;
		}

		// set the compensation force and torque for the device
		phState->phGravityForce[0] = gravityK*gravityForceVector[0] + F[0];
		phState->phGravityForce[1] = gravityK*gravityForceVector[1] + F[1];
		phState->phGravityForce[2] = gravityK*gravityForceVector[2] + F[2];

		phState->phGravityTorqe[0] = gravityK*gravityForceVector[3];
		phState->phGravityTorqe[1] = gravityK*gravityForceVector[4];
		phState->phGravityTorqe[2] = gravityK*gravityForceVector[5];


		// set the control compensation force + cubic limits

		hdSetDoublev(HD_CURRENT_FORCE, phState->phGravityForce);
		hdSetDoublev(HD_CURRENT_TORQUE, phState->phGravityTorqe);

	}

	//////////////////////////////////////////////////////////////////////
	// NO GRAVITY COMPENSATION
	else{

		phState->phGravityForce[0] = F[0];
		phState->phGravityForce[1] = F[1];
		phState->phGravityForce[2] = F[2];

		phState->phGravityTorqe[0] = 0;
		phState->phGravityTorqe[1] = 0;
		phState->phGravityTorqe[2] = 0;

		// set the control compensation force + cubic limits
		
		hdSetDoublev(HD_CURRENT_FORCE, phState->phGravityForce);
		hdSetDoublev(HD_CURRENT_TORQUE, phState->phGravityTorqe);

	}

	hdEndFrame(hdGetCurrentDevice());

	//Execute this for ever. Use HD_CALLBACK_ONCE for one time check
	return HD_CALLBACK_CONTINUE;
}


