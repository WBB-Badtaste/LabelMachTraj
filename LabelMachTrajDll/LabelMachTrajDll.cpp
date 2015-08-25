// LabelMachTrajDll.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "LabelMachTrajDll.h"


// This is an example of an exported variable
LABELMACHTRAJDLL_API int nLabelMachTrajDll=0;

// This is an example of an exported function.
LABELMACHTRAJDLL_API int fnLabelMachTrajDll(void)
{
	return 42;
}

// This is the constructor of a class that has been exported.
// see LabelMachTrajDll.h for the class definition
CLabelMachTrajDll::CLabelMachTrajDll()
{
	return;
}
