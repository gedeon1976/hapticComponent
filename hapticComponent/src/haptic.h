/***************************************************************************
*   Copyright (C) 2007 by Emmanuel Nu�o                                   *
*   emmanuel.nuno@upc.edu                                                 *
*
*   modified by Henry Portilla 2015
*
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

#pragma once

#include "InterfaceHaptic.hpp"
#include <iostream>


HDCallbackCode HDCALLBACK hdState(void *);

class Haptic : public InterfaceHaptic {
public:
	/// create the interface implementing the virtual constructor
	InterfaceHaptic *Create() { return new Haptic(); };
	~Haptic(){};

	void Init(bool &init);
	void startConnection();	
	bool setWorkSpaceLimits(mt::Vector3 minCubeLimits, mt::Vector3 maxCubeLimits);
	void getWorkSpaceLimits(mt::Vector3 minCubicLimits, mt::Vector3 maxCubicLimits);
	void getHapticPosition(mt::Transform &hapticPosition);
	void getGravityCompensation(CompensationForce &forceVector);
	void setGravityCompensation(bool &gravityEnable);
	bool setForce(const Vect6);
	void setKa(double Ka);
	void setKd(double kd);
	bool getJacobian(ublas::matrix<mt::Scalar> &Jacobian);
	bool getJacobianTranspose(ublas::matrix<mt::Scalar> &JacobianT);
	void closeConnection();


private:
	HHD m_hHD;
	HDErrorInfo m_error;
	HapticState *m_phState;
	bool m_init;

	int hapticDeviceID;
	bool hapticStatus;
	mt::Transform HapticPosition;
	mt::Vector3 position;
	mt::Rotation orientation;
	

	/// get the haptic device type
	void getHapticDevice(int &hapticType);

	/// calibrate the haptic device
	bool calibrate();

	/// start the haptic device
	bool start();

	/// stop the haptic device
	bool stop();

	/// set the gravity compensation
	void setGravityVector(jointAngles &currentAngles, CompensationForce &currentForce);

	/// old Function, not used	
	bool getPositionTX(mt::Transform &);

	/// get the position Xh,Yh,Zh on haptic coordinates
	/// @param[in,out]	mt::Transform it saves the 3 vector haptic position
	bool getPosition(mt::Transform &);

	/// get the velocity of each joint
	/// @param[in,out] Vect6 contains the speeds of haptic Pose and torque joints
	bool getVelocity(Vect6 &);

	/// get the state of the stylus button
	bool getButton();

	/// get the position of the joints
	/// @param[in,out] Vect6 it contains the position of each joint of the haptic
	bool getJointPosition(Vect6 &);

	/// get the position of the joints overloaded method
	/// @param[in,out] currentAngles it contains the position of each joint of the haptic
	bool getJointPosition(jointAngles &currentAngles);

	/// set the torque to each torque
	/// @param[in,out] Vect6 it contains the torque to be set at each joint motor
	bool setMotorTorque(const Vect6);

	

	
};




