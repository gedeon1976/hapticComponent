/*!  \brief

This class is the access door to the haptic interface
following the factory method pattern design for the API access

the next lines shows an example of how to use this component
to get access to its functionalities

\code{.cpp}


		//This project test the haptic Component
		//created for the purpose of connect a haptic device
		//the device is a phantom 1.5 6 d.o.f High Force
		
		#include "HapticAccess.h"
		
		using namespace std;
		
		int main(int argc, char ** argv)
		{
		// create the component using the factory method pattern
		HapticAccess AccessObject;
		InterfaceHaptic *HapticComponent = AccessObject.CreateHaptic();

		// Initialize component
		bool init = true;
		HapticComponent->Init(init);

		// create a camera state
		
			// define OutputArrays to save the results of the StereoCamera Component
		

		// call the methods from the Haptic Component

		// check the camera state

		
		// call the calibration process
		StereoComponent->calibrateStereoCamera(leftCameraSettingsFile, rightCameraSettingsFile);

		}

		CameraCalibrationStatus = StereoComponent->getStereoCameraState();


		if (CameraCalibrationStatus == InterfaceStereoCamera::STEREO_CALIBRATED)
		{
		// get the results
		

		// perform a tracking test to proof the results
		
		}

		delete HapticComponent;

		return 0;
		}

\endcode


*/


#include "haptic.h"			// Interface API

class HapticAccess
{

public:
	/// Create the Haptic component calling this factory method
	InterfaceHaptic *CreateHaptic();

};