//This project test the haptic Component
//created with the purpose of connect a haptic device
//the device is a phantom 1.5 6 d.o.f High Force
		
	#include "HapticAccess.h"
	
	using namespace std;
	
	int main(int argc, char ** argv)
	{
	// create the component using the factory method pattern
	HapticAccess AccessObject;
	InterfaceHaptic *HapticComponent = AccessObject.CreateHaptic();

	bool enableGravity = true;
	mt::Transform hapticPosition;
	mt::Vector3 position;

	// Initialize component
	bool init = true;
	HapticComponent->Init(init);

	// call the methods from the Haptic Component
	HapticComponent->startConnection();
	HapticComponent->setGravityCompensation(enableGravity)

	while(1){

		// get the haptic position
		HapticComponent->getHapticPosition(hapticPosition);
		position = hapticPosition.getTraslation();
		cout << "X: " << position[0] << "Y: " << position[1] << "Z: " << position[2] << "\n" << endl;			

	}
	
	delete HapticComponent;

	return 0;
	}