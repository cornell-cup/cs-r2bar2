#pragma once

// Windows Header Files
#include <windows.h>
#include <Shlobj.h>

// Safe Release
template<class Interface> inline void SafeRelease(Interface *& kInterface) {
	if (kInterface != NULL) {
		kInterface->Release();
		kInterface = NULL;
	}
}

class KinectInterface
{
public:
	// Constructor
	KinectInterface(int inColorWidth, int inColorHeight, int inDepthWidth, int inDepthHeight);

	// Destructor
	virtual ~KinectInterface();

	// Get the color buffer (size = colorWidth * colorHeight)
	virtual UINT32 * getColorBuffer();

	// Get the depth buffer (size = depthWidth * depthHeight)
	virtual UINT16 * getDepthBuffer();

	// Color buffer width
	int colorWidth;
	// Color buffer height
	int colorHeight;
	// Depth buffer width
	int depthWidth;
	// Depth buffer height
	int depthHeight;
};

