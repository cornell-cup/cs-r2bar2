#pragma once
#include "KinectInterface.h"
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

class Kinect1 :
	public KinectInterface
{
public:
	// Initialize Kinect 1
	Kinect1();

	// Dispose Kinect 1
	~Kinect1();

	// Get the color buffer (size = colorWidth * colorHeight)
	UINT32 * getColorBuffer();

	// Get the depth buffer (size = depthWidth * depthHeight)
	UINT16 * getDepthBuffer();
	
private:
	// Current Kinect 1
	INuiSensor * sensor;

	// Reader
	HANDLE colorStream;
	HANDLE depthStream;

	// Cached Buffers
	UINT32 * colorBuffer;
	UINT16 * depthBuffer;
};

