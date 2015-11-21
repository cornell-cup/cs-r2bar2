#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <time.h>
#include <math.h>
#include <iostream>
#include <thread>
#include <Windows.h>

#include "R2Bar2.h"
#include "Kinect1.h"

#define PI 3.14159f

using namespace cv;

float getHue(int red, int green, int blue)
{

	float min = (float) min(min(red, green), blue);
	float max = (float) max(max(red, green), blue);

	float hue = 0.f;
	if (max == red)
	{
		hue = (green - blue) / (max - min);

	}
	else if (max == green)
	{
		hue = 2.f + (blue - red) / (max - min);

	}
	else
	{
		hue = 4.f + (red - green) / (max - min);
	}

	hue = hue / 6.f;
	if (hue < 0.f) hue += 1.f;

	return hue;
}

void frameToDepth(int i, int j, int & offi, int & offj)
{
	double scale = 1.0;
	offi = (int) ((i - 30) * scale);
	offj = (int) ((j + 0) * scale);
}

void depthToFrame(int i, int j, int & offi, int & offj)
{
	double scale = 1.0;
	offi = (int) (i / scale + 30);
	offj = (int) (j / scale);
}

void frameToColor(int i, int j, int & offi, int & offj)
{
	offi = i * 2;
	offj = j * 2;
}

FLOAT barcodePositions[10][2] = {
	{ 0.f, 0.6f },
	{ 0.f, -0.6f },
	{ 0.f, 0.f },
	{ 0.f, 0.f },
	{ 0.f, 0.f },
	{ 0.f, 0.f },
	{ 0.f, 0.f },
	{ 0.f, 0.f },
	{ 0.f, 0.f },
	{ 0.f, 0.f }
};

// Whatever
std::vector<std::vector<Point2f>> barcodePoints;
BOOL capturing = false;

// Position
const UINT32 posBuffer = 10;
UINT32 posIndex = 0;
FLOAT posx[posBuffer] = { 0.f };
FLOAT posy[posBuffer] = { 0.f };

void readBarcode(int width, int height, UINT32 * rows, const UINT16 * depthBuffer)
{
	capturing = true;

	// List of values
	std::vector<int> col;
	std::vector<int> val;

	// Accumulate data
	// Assume barcodes are labeled 0 to 9 (thousands digit)
	// Count number read
	INT32 counts[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	// Sum total pixel locations
	INT32 totalp[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };;
	// Sum total angle vectors
	FLOAT totalx[10] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
	FLOAT totaly[10] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
	// Min/max
	INT32 minx[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	INT32 maxx[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	INT32 miny[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	INT32 maxy[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

	// Decoder result
	R2Bar2::Result result;
	for (int i = 0; i < height; ++i)
	{
		// Run lengths
		std::vector<UINT32> rle;
		R2Bar2::argbToMonoRLE(width, rows + i * width, rle);
		// Run totals
		std::vector<UINT32> rls;
		UINT32 sum = 0;
		for (UINT32 j = 0; j < rle.size(); ++j)
		{
			rls.push_back(sum);
			sum += rle[j];
		}
		rls.push_back(sum);

		INT32 read = R2Bar2::decode(rle, 0, result);
		if (read >= 0 && result.digits == 4)
		{
			val.push_back(result.value);
			INT32 col = result.value / 1000;
			INT32 a2 = (result.value / 100) % 10;
			INT32 a1 = (result.value / 10) % 10;
			INT32 a0 = (result.value / 1) % 10;
			if (a2 % 2 == 1)
			{
				a1 = 9 - a1;
			}
			if (a1 % 2 == 1)
			{
				a0 = 9 - a0;
			}
			FLOAT ang = (a2 * 100 + a1 * 10 + a0) / 200.f * 2.f * PI;
			counts[col] += 1;
			totalp[col] += i;
			totalx[col] += cos(ang);
			totaly[col] += sin(ang);
			if (minx[col] == -1)
			{
				minx[col] = i;
			}
			maxx[col] = i;
			if (miny[col] == -1 || rls[result.start] < miny[col])
			{
				miny[col] = rls[result.start];
			}
			if (maxy[col] == -1 || rls[result.end] > maxy[col])
			{
				maxy[col] = rls[result.end];
			}
			//printf("%d %d %d\n", i, result.value, result.digits);
		}
	}
	//printf("Found %d\n", val.size());

	// Accumulate data and come up with a result
	barcodePoints.clear();
	for (int i = 0; i < 10; ++i)
	{
		if (counts[i] >= 10)
		{
			FLOAT dx = (FLOAT) totalx[i] / (FLOAT) counts[i];
			FLOAT dy = (FLOAT) totaly[i] / (FLOAT) counts[i];
			FLOAT angle = atan2f(dy, dx) * 180.f / PI;
			// Adjust offset in image using fov of 62
			//FLOAT offset = ((maxx[i] + minx[i]) * 0.5f - width * 0.5f) / (FLOAT) width * 62.f;
			printf("%d %d %f %f %f\n", i, counts[i], angle, dx, dy);
			printf("    %d %d %d %d\n", minx[i], maxx[i], miny[i], maxy[i]);

			// Assign position
			INT32 mx = (minx[i] + maxx[i]) / 4;
			INT32 my = (miny[i] + maxy[i]) / 4;
			
			FLOAT distance = 1.f;
			if (mx >= 0 && mx < 640 && my >= 0 && my < 480)
			{
				distance = (FLOAT) depthBuffer[my * 640 + mx] / 1000.f;
			}
			posx[posIndex] = barcodePositions[i][0] + distance * dx;
			posy[posIndex] = barcodePositions[i][1] + distance * dy;
			posIndex = (posIndex + 1) % posBuffer;

			// Assign box
			std::vector<Point2f> points;
			points.push_back(Point2f((FLOAT) minx[i] * 0.5f, (FLOAT) miny[i] * 0.5f));
			points.push_back(Point2f((FLOAT) maxx[i] * 0.5f, (FLOAT) miny[i] * 0.5f));
			points.push_back(Point2f((FLOAT) maxx[i] * 0.5f, (FLOAT) maxy[i] * 0.5f));
			points.push_back(Point2f((FLOAT) minx[i] * 0.5f, (FLOAT) maxy[i] * 0.5f));
			points.push_back(Point2f((FLOAT) minx[i] * 0.5f, (FLOAT) miny[i] * 0.5f));
			barcodePoints.push_back(points);
		}
	}

	capturing = false;
}

int main()
{
	// OpenCV Window
	namedWindow("Feed", CV_WINDOW_AUTOSIZE);
	namedWindow("Position", CV_WINDOW_AUTOSIZE);

	// Setup Kinect
	KinectInterface * kinect;
	kinect = new Kinect1();

	// Timing
	clock_t start = clock();
	clock_t finish = clock();

	int wwidth = kinect->colorWidth / 2;
	int wheight = kinect->colorHeight / 2;
	BYTE * buffer = new BYTE[wwidth * wheight * 4];

	// Obstacles
	const int numFrames = 5;
	BOOL ** obsFrames = new BOOL*[numFrames];
	for (int i = 0; i < numFrames; i++)
	{
		obsFrames[i] = new BOOL[kinect->depthWidth * kinect->depthHeight];
	}
	int obsFrame = 0;
	BOOL * obsSum = new BOOL[kinect->depthWidth * kinect->depthHeight];

	// Barcodes
	const int captureTime = 30;
	int captureBarcode = 0;

	while (1)
	{
		UINT32 * colorBuffer = kinect->getColorBuffer();
		UINT16 * depthBuffer = kinect->getDepthBuffer();
		if (colorBuffer != NULL)
		{
			// Display
			for (int i = 0; i < wheight; i++)
			{
				for (int j = 0; j < wwidth; j++)
				{
					int oi, oj;
					frameToColor(i, j, oi, oj);
					UINT32 c = colorBuffer[oi * kinect->colorWidth + oj];
					buffer[(i * wwidth + j) * 4 + 0] = (c >> 0) & 0xFF;
					buffer[(i * wwidth + j) * 4 + 1] = (c >> 8) & 0xFF;
					buffer[(i * wwidth + j) * 4 + 2] = (c >> 16) & 0xFF;
					buffer[(i * wwidth + j) * 4 + 3] = (c >> 24) & 0xFF;
				}
			}
		}
		if (depthBuffer != NULL)
		{
			for (int i = 0, l = kinect->depthWidth * kinect->depthHeight; i < l; i++)
			{
				obsFrames[obsFrame][i] = (depthBuffer[i] > 0 && depthBuffer[i] <= 1000);

				BOOL sum = true;
				for (int j = 0; j < numFrames; j++)
				{
					sum = sum && obsFrames[j][i];
				}
				obsSum[i] = sum;
			}
			obsFrame = (obsFrame + 1) % numFrames;

			// Display
			for (int i = 0; i < wheight; i++)
			{
				for (int j = 0; j < wwidth; j++)
				{
					int offi, offj;
					frameToDepth(i, j, offi, offj);
					if (offi >= 0 && offi < kinect->depthHeight &&
						offj >= 0 && offj < kinect->depthWidth &&
						obsSum[offi * kinect->depthWidth + offj])
					{
						buffer[(i * wwidth + j) * 4 + 2] = 255;
					}
				}
			}
		}

		// If both buffers are not null and want to get barcode
		if (colorBuffer != NULL && depthBuffer != NULL && !capturing)//captureBarcode == captureTime)
		{
			UINT32 * transpose = new UINT32[kinect->colorWidth * kinect->colorHeight];
			R2Bar2::transpose(kinect->colorWidth, kinect->colorHeight, colorBuffer, transpose);
			readBarcode(kinect->colorHeight, kinect->colorWidth, transpose, depthBuffer);
			free(transpose);
			//std::thread br(readBarcode, kinect->colorHeight, kinect->colorWidth, transpose);
			//br.detach();
		}

		Mat frame(wheight, wwidth, CV_8UC4, buffer);
		Mat posframe(1000, 1000, CV_8UC4);

		// Draw barcode
		if (barcodePoints.size() > 0)
		{
			for (UINT32 i = 0; i < barcodePoints.size(); ++i)
			{
				if (barcodePoints[i].size() == 5)
				{
					for (UINT32 j = 0; j < 4; ++j)
					{
						line(frame, barcodePoints[i][j], barcodePoints[i][j + 1], Scalar(255, 0, 0), 3);
					}
				}
			}
		}

		// Average and draw position
		circle(posframe, Point2f(500, 400), 18, Scalar(0, 0, 255), -1);
		circle(posframe, Point2f(500, 600), 18, Scalar(0, 0, 255), -1);
		FLOAT avgx = 0.f;
		FLOAT avgy = 0.f;
		for (UINT32 i = 0; i < posBuffer; ++i)
		{
			avgx += posx[i];
			avgy += posy[i];
		}
		avgx = avgx / (FLOAT) posBuffer;
		avgy = avgy / (FLOAT) posBuffer;
		circle(posframe, Point2f(avgx * 200 + 500, 500 - avgy * 200), 10, Scalar(255, 0, 0), -1);

		imshow("Feed", frame);
		imshow("Position", posframe);

		int duration = (int) ((finish - start) * 1000.0 / CLOCKS_PER_SEC);
		//printf("%2d ms\n", duration);

		int wait = 33 - duration;
		if (wait <= 0) wait = 1;
		int key = waitKey(wait);
		if (key == 27)
		{
			break;
		}
		else if (key == 32)
		{
			captureBarcode = captureTime;
		}
	}

	return 0;
}