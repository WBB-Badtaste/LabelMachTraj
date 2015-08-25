// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the LABELMACHTRAJDLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// LABELMACHTRAJDLL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef LABELMACHTRAJDLL_EXPORTS
#define LABELMACHTRAJDLL_API __declspec(dllexport)
#else
#define LABELMACHTRAJDLL_API __declspec(dllimport)
#endif

// This class is exported from the LabelMachTrajDll.dll
// class LABELMACHTRAJDLL_API CLabelMachTrajDll {
// public:
// 	CLabelMachTrajDll(void);
// 	// TODO: add your methods here.
// };
// 
// extern LABELMACHTRAJDLL_API int nLabelMachTrajDll;

LABELMACHTRAJDLL_API BOOL InitLabelMachTrajDll(void);

LABELMACHTRAJDLL_API BOOL MoveLabelMachTraj(void);

LABELMACHTRAJDLL_API BOOL SetMachTrajPars(void);
