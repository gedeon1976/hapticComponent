//This project test the haptic Component
//created with the purpose of connect a haptic device
//the device is a phantom 1.5 6 d.o.f High Force
		
	#include "HapticAccess.h"
	#include <opencv2\highgui.hpp>
    #include <conio.h>	

	using namespace std;
	
	int main(int argc, char ** argv)
	{
	// create the component using the factory method pattern
	HapticAccess AccessObject;
	InterfaceHaptic *HapticComponent = AccessObject.CreateHaptic();

	bool enableGravity = true;
	mt::Transform hapticPosition;
	mt::Vector3 position;
	CompensationForce gravityForce;
	ublas::matrix<mt::Scalar> Jacobian;
	ublas::matrix<mt::Scalar> JacobianT;
	Vect6 controlTorqueForce,gravityForceVector(6);

	// set the workspace Limits for the haptic, for us it is a cube
	// the measures are relatives to Haptic own coordinates
	mt::Vector3 MinCubicLimits, MaxCubicLimits;
	float Xmin = -150;
	float Xmax = 150;
	float Ymin = 0;
	float Ymax = 300;
	float Zmin = -80;
	float Zmax = 80;

	MinCubicLimits[0] = Xmin; MaxCubicLimits[0] = Xmax;
	MinCubicLimits[1] = Ymin; MaxCubicLimits[1] = Ymax;
	MinCubicLimits[2] = Zmin; MaxCubicLimits[2] = Zmax;

	// Initialize component
	bool init = true;
	HapticComponent->Init(init);

	// call the methods from the Haptic Component
	HapticComponent->startConnection();
	HapticComponent->setWorkSpaceLimits(MinCubicLimits, MaxCubicLimits);
	HapticComponent->setGravityCompensation(enableGravity);

	// test Haptic gains
	double Ka = 1000;
	double Kd = 0.001;
	int ka_in, kd_in;
	int key;

	while (1)
	{
					
		key = getch();	

	
		switch (key)
		{
		case 97:
			Ka = Ka + 25;
			std::cout << "Ka Value= %f \n" << Ka << std::endl;
			break;
		case 115:
			Ka = Ka - 25;
			std::cout << "Ka Value=  %f \n" << Ka << std::endl;
			break;
		case 100:
			Kd = Kd + 0.0001;
			std::cout << "Kd Value= %f \n" << Kd << std::endl;
			break;
		case 102:
			Kd = Kd - 0.0001;
			std::cout << "Kd Value= %f" << Kd << std::endl;
			break;
		}

		HapticComponent->setKa(Ka);
		HapticComponent->setKd(Kd);		
	
		
		
		// get the haptic position
		HapticComponent->getHapticPosition(hapticPosition);
		position = hapticPosition.getTranslation();
		cout << " X: " << position[0] << " Y: " << position[1] << " Z: " << position[2] << "\n" << endl;

		//HapticComponent->getGravityCompensation(gravityForce);
		//cout << "compensation gravity Force \n" << endl;
		/* cout << " f1: " << gravityForce.f1 << " f2: " << gravityForce.f2 <<
			    " f3: " << gravityForce.f3 << " f4: " << gravityForce.f4 <<
			    " f5: " << gravityForce.f5 << " f6: " << gravityForce.f6 << "\n" << endl;
				*/
		// set the control compensation force
		//HapticComponent->getJacobian(Jacobian);
		//HapticComponent->getJacobianTranspose(JacobianT);

		////set the static force t = JacobianT*gravityForce;
		//gravityForceVector[0] = gravityForce.f1;
		//gravityForceVector[1] = gravityForce.f2;
		//gravityForceVector[2] = gravityForce.f3;
		//gravityForceVector[3] = gravityForce.f4;
		//gravityForceVector[4] = gravityForce.f5;
		//gravityForceVector[5] = gravityForce.f6;

		//controlTorqueForce = ublas::prod(JacobianT, gravityForceVector);
		//HapticComponent->setForce(controlTorqueForce);
	
	}
	
	delete HapticComponent;

	return 0;
	}