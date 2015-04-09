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
	~Haptic();

	void Init(bool &init);
	bool calibrate();
	bool start();
	bool stop();
	bool getPositionTX(mt::Transform &);
	bool getPosition(mt::Transform &);
	bool getVelocity(Vect6 &);
	bool setForce(const Vect6);
	bool getButtom();
	bool getJointPosition(Vect6 &);
	bool setMotorTorque(const Vect6);
	bool getJacobian(ublas::matrix<mt::Scalar> &);
	bool getJacobianTranspose(ublas::matrix<mt::Scalar> &);
	bool SetWorkSpaceLimits(mt::Vector3 minCubeLimits, mt::Vector3 maxCubeLimits);


public:
	HHD m_hHD;
	HDErrorInfo m_error;
	HapticState *m_phState;
	bool m_init;
};




