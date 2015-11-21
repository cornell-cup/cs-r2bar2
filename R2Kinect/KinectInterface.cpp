#include "KinectInterface.h"

KinectInterface::KinectInterface(int inColorWidth, int inColorHeight, int inDepthWidth, int inDepthHeight) :
	colorWidth(inColorWidth), colorHeight(inColorHeight), depthWidth(inDepthWidth), depthHeight(inDepthHeight)
{
}

KinectInterface::~KinectInterface()
{
}

UINT32 * KinectInterface::getColorBuffer()
{
	return NULL;
}

UINT16 * KinectInterface::getDepthBuffer()
{
	return NULL;
}
