#include "HapticAccess.h"

InterfaceHaptic *HapticAccess::CreateHaptic()
{
	InterfaceHaptic *Obj = new Haptic();
	return Obj;
}