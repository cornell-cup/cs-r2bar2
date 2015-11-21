#include "Kinect1.h"

Kinect1::Kinect1() :
	KinectInterface(1280, 960, 640, 480), sensor(NULL),
	colorBuffer(new UINT32[colorWidth * colorHeight]), depthBuffer(new UINT16[depthWidth * depthHeight])
{
	int numSensors, result;

	// Make sure sensor exists
	result = NuiGetSensorCount(&numSensors);
	if (result >= 0 && numSensors > 0)
	{
		result = NuiCreateSensorByIndex(0, &sensor);
		if (result >= 0)
		{
			// Initialize sensor with both color and depth
			sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);

			// Open color stream
			sensor->NuiImageStreamOpen(
				// 1280 x 960 color
				NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_1280x960,
				// Do not use near mode
				0,
				// Buffer for 2 frames
				2,
				// No handler for events
				NULL,
				// Get the color stream
				&colorStream);

			// Open depth stream
			sensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
				NUI_IMAGE_RESOLUTION_640x480,
				// Change this argument to NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE for near mode
				0,
				2,
				NULL,
				&depthStream);
		}
	}
}


Kinect1::~Kinect1()
{
	// Shutdown the sensor
	if (sensor)
	{
		sensor->NuiShutdown();
	}
	// Close color stream
	if (colorStream != INVALID_HANDLE_VALUE)
	{
		CloseHandle(colorStream);
	}
	// Close depth stream
	if (depthStream != INVALID_HANDLE_VALUE)
	{
		CloseHandle(depthStream);
	}

	SafeRelease(sensor);
}

UINT32 * Kinect1::getColorBuffer()
{
	int result;

	// Grab an image frame
	NUI_IMAGE_FRAME imageFrame;
	result = sensor->NuiImageStreamGetNextFrame(colorStream, 0, &imageFrame);
	// Make sure the frame is not empty
	if (result >= 0)
	{
		// Get frame texture
		NUI_LOCKED_RECT lockedRect;
		INuiFrameTexture * texture = imageFrame.pFrameTexture;
		texture->LockRect(0, &lockedRect, NULL, 0);

		UINT32 * bufferIndex = colorBuffer;
		BYTE * frameIndex = (BYTE *) lockedRect.pBits;
		BYTE * end = frameIndex + (colorWidth * colorHeight) * 4;

		// Copy data to buffer
		while (frameIndex < end)
		{
			// 4 byte BGRA to 1 int ARGB
			*bufferIndex =
				frameIndex[0] |
				(frameIndex[1] << 8) |
				(frameIndex[2] << 16) |
				(frameIndex[3] << 24);
			bufferIndex++;
			frameIndex += 4;
		}

		// Release
		texture->LockRect(0, &lockedRect, NULL, 0);
		sensor->NuiImageStreamReleaseFrame(colorStream, &imageFrame);
	}

	return colorBuffer;
}

UINT16 * Kinect1::getDepthBuffer()
{
	int result;

	// Grab an image frame
	NUI_IMAGE_FRAME imageFrame;
	result = sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame);
	// Make sure the frame is not empty
	if (result >= 0)
	{
		// Get frame texture
		NUI_LOCKED_RECT lockedRect;
		INuiFrameTexture * texture = imageFrame.pFrameTexture;
		texture->LockRect(0, &lockedRect, NULL, 0);

		UINT16 * bufferIndex = depthBuffer;
		UINT16 * frameIndex = (UINT16 *) lockedRect.pBits;
		UINT16 * end = frameIndex + (depthWidth * depthHeight);

		// Copy data to buffer
		while (frameIndex < end)
		{
			// 4 byte BGRA to 1 int ARGB
			*bufferIndex = NuiDepthPixelToDepth(*frameIndex);
			bufferIndex++;
			frameIndex++;
		}

		// Release
		texture->LockRect(0, &lockedRect, NULL, 0);
		sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
	}

	return depthBuffer;
}

