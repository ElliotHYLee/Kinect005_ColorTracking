// General headers
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>     /* abs */
#include <iostream>
#include <iomanip>
#include <chrono>
// Serial Communication
#include "serialcomm.h"
#include <string>

// OpenNI2 headers
#include <OpenNI.h>
#include <NiTE.h>
#include <conio.h>

//OpenCV headers
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


//Open Gl Header
#include "GL\glut.h"

using namespace openni;
using namespace cv;


// memeber variables

int relX, relY, relZ;
float actX, actY, actZ, prevActX, prevActY, prevActZ;

double lpTimeStep;
float actZArr[5];


int window_w = 640;
int window_h = 480;

Mat imgLines;
int iLastX;
int iLastY;

int iLowH;
int iHighH;

int iLowS;
int iHighS;

int iLowV;
int iHighV;

Device device;
VideoStream colorSensor;
VideoStream depthSensor;
VideoFrameRef newFrame_depth;
VideoFrameRef newFrame_color;
Status status = STATUS_OK;

//usb
CSerialComm serialComm;
std::string messageX, messageY, messageZ;

long timeDiff;


OniRGB888Pixel* gl_texture;
bool histogram_enable = true;
bool color_enable = false;
bool blackfill_enable = false;

uint64_t dOld, dNew, dDt, cOld, cNew, cDt, dcDt[100], avg, maxavg;



// general function headers
void initiateUSBCommunication();
void sendCoordinateMsg();


char ReadLastCharOfLine();
bool HandleStatus(Status status);
int prepareKinect();
int startDepthStream();
int startColorStream();
int primaryLoop(int a, _TCHAR* b[]);
double glMin(double a, double b);
void depthMonitor(VideoFrameRef newFrame);
void checkTimeDifference(VideoFrameRef c, VideoFrameRef d);
void depthFrameProcess(VideoFrameRef depthFrame);
void colorFrameProcess(VideoFrameRef colorFrame);
void colorDetection(Mat *img);

//openGL headers
void gl_KeyboardCallback(unsigned char key, int x, int y);
void gl_IdleCallback();
void gl_DisplayCallback();


int _tmain(int argc, _TCHAR* argv[])
{

	messageX = "hi";
	messageY = "hi";
	messageZ = "hi";

	lpTimeStep = 0;

	relX = 0;
	relY = 0;
	relZ = 0;
	actX = 0;
	actY = 0;
	actZ = 0;
	prevActX = 0;
	prevActY = 0;
	prevActZ = 0;

	iLowH = 156;
	iHighH = 179;

	iLowS = 112;
	iHighS = 255;

	iLowV = 30;
	iHighV = 255;
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
												//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);


	maxavg = 0;
	for (int i = 0; i < 100; i++)
	{
		dcDt[i] = 0;
	}


	//usb
	initiateUSBCommunication();


	// kinect connection
	if (prepareKinect()) return 1;
	// depth stream starts
	if (startDepthStream()) return 1;
	// color stream starts
	if (startColorStream()) return 1;


	primaryLoop(argc, argv);


	return 0;




}

void initiateUSBCommunication()
{
	// SerialPort Polling

	int baudRate = 115200;
	string a = "COM00";
	char *cstr = new char[a.length() + 1];
	int comPortNumber = 0;
	for (comPortNumber = 0; comPortNumber < 30; comPortNumber++)
	{
		a = "COM" + std::to_string(comPortNumber);
		strcpy_s(cstr, a.length() + 1, a.c_str());
		if (serialComm.connect(cstr, baudRate))
		{
			cout << "COM" << comPortNumber << " is available." << endl;
			serialComm.disconnect();
		}
	}

	cout << "Type the port number to send msg ex) 1" << endl;
	cin >> comPortNumber;

	// Serial Port Connection
	a = "COM" + std::to_string(comPortNumber);
	strcpy_s(cstr, a.length() + 1, a.c_str());
	if (serialComm.connect(cstr, baudRate))
	{
		cout << "connected" << endl;
	}
	else
	{
		cout << "connection failed" << endl;
	}
	delete[] cstr;
}

char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do
	{
		lastChar = newChar;
		newChar = getchar();
	} while ((newChar != '\n') && (newChar != EOF));
	return (char)lastChar;
}

bool HandleStatus(Status status)
{
	if (status == STATUS_OK) return true;
	printf("ERROR: #%d, %s", status, OpenNI::getExtendedError());
	ReadLastCharOfLine();
	return false;
}

int prepareKinect()
{
	printf("\r\n---------------------- Init OpenNI --------------------------\r\n");
	printf("Scanning machine for devices and loading modules/drivers ...\r\n");

	// software-wise preparation
	status = OpenNI::initialize();
	if (!HandleStatus(status)) return 1;
	printf("Completed.\r\n");

	printf("\r\n---------------------- Open Device --------------------------\r\n");
	printf("Opening first device ...\r\n");


	// open any physical Kinect connected to this computer
	status = device.open(ANY_DEVICE);
	if (!HandleStatus(status)) return 1;
	printf("%s Opened, Completed.\r\n", device.getDeviceInfo().getName());


	status = device.setDepthColorSyncEnabled(true);
	if (!HandleStatus(status)) return 1;
	printf("%s DepthColorSyncEnabled.\r\n", device.getDeviceInfo().getName());

	return 0;
}

int startDepthStream()
{
	printf("\r\n---------------------- Depth Stream --------------------------\r\n");
	printf("Checking depth stream is connected\r\n");
	if (!device.hasSensor(SENSOR_DEPTH))
	{
		printf("Stream not supported by this device.\r\n");
		return 1;
	}

	// ready to connect depth sensor

	printf("Connecting to a depth sensor... \r\n");
	status = depthSensor.create(device, SENSOR_DEPTH);
	if (!HandleStatus(status)) return 1;

	// setting frame rate and size
	printf("Setting video mode to 640x480x30 Depth 1MM ...\r\n");
	VideoMode vmod_depth;
	vmod_depth.setFps(25);
	vmod_depth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	vmod_depth.setResolution(640, 480);
	status = depthSensor.setVideoMode(vmod_depth);
	if (!HandleStatus(status)) return 1;
	printf("Done.\r\n");


	// connect to the depth sensor
	printf("Starting depth stream \r\n");
	status = depthSensor.start();
	if (!HandleStatus(status)) return 1;
	printf("Done.\r\n");

	return 0;
}

int startColorStream()
{
	printf("\r\n---------------------- Image Stream --------------------------\r\n");
	printf("Checking if stream is supported ...\r\n");
	if (!device.hasSensor(SENSOR_COLOR))
	{
		printf("Stream not supported by this device.\r\n");
		return 1;
	}

	printf("Asking device to create a color stream ...\r\n");
	status = colorSensor.create(device, SENSOR_COLOR);
	if (!HandleStatus(status)) return 1;

	VideoMode vmod_color;
	vmod_color.setFps(25);
	vmod_color.setPixelFormat(PIXEL_FORMAT_RGB888);
	vmod_color.setResolution(640, 480);
	status = colorSensor.setVideoMode(vmod_color);


	printf("Starting stream ...\r\n");
	status = colorSensor.start();
	if (!HandleStatus(status)) return 1;
	printf("Done.\r\n");

	return 0;
}

int primaryLoop(int a, _TCHAR* b[])
{
	printf("\r\n---------------------- OpenGL -------------------------\r\n");
	printf("Initializing OpenGL ...\r\n");
	gl_texture = (OniRGB888Pixel*)malloc(window_w * window_h * sizeof(OniRGB888Pixel));
	glutInit(&a, (char**)b);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(window_w, window_h);
	glutCreateWindow("OpenGL | OpenNI 2.x CookBook Sample");
	glutKeyboardFunc(gl_KeyboardCallback);
	glutDisplayFunc(gl_DisplayCallback);
	glutIdleFunc(gl_IdleCallback);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	printf("Starting OpenGL rendering process ...\r\n");
	glutMainLoop();



	return 0;
}

//openGL functions
void gl_KeyboardCallback(unsigned char key, int x, int y)
{
	if (key == 27) // ESC Key
	{
		depthSensor.destroy();
		device.close();
		OpenNI::shutdown();
		//exit(0);
	}
	else if (key == 72 || key == 104) // H or h key
	{
		histogram_enable = !histogram_enable;
		color_enable = false;
	}
	else if (key == 70 || key == 102) // F or f key
	{
		blackfill_enable = !blackfill_enable;
	}
	else if (key == 67 || key == 99) // C or c key
	{
		color_enable = !color_enable;
		histogram_enable = false;
	}
}

void gl_IdleCallback()
{
	glutPostRedisplay();
}

void gl_DisplayCallback()
{
	if (depthSensor.isValid() && colorSensor.isValid())
	{
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		// get depth and color image
		status = depthSensor.readFrame(&newFrame_depth);
		status = colorSensor.readFrame(&newFrame_color);

		//checkTimeDifference(newFrame_color, newFrame_depth);

		// all algorithm for color process
		colorFrameProcess(newFrame_color);

		// all algorithm for depth process
		depthFrameProcess(newFrame_depth);
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

		double duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();


		lpTimeStep = 0.95*lpTimeStep + 0.05*duration;

		//std::cout << lpTimeStep << " milli second " << 1000/lpTimeStep << " Hz \n";
	}
}

void checkTimeDifference(VideoFrameRef c, VideoFrameRef d)
{
	dNew = d.getTimestamp();

	dDt = dNew - dOld;
	std::cout << "depth: Dt = " << (double)dDt << "  freq = " << (double) 1000000.0 / dDt << "\n";
	dOld = dNew;

	cNew = c.getTimestamp();
	cDt = cNew - cOld;
	std::cout << "color: Dt = " << (double)cDt << "  freq = " << (double)1000000.0 / cDt << "\n";
	cOld = cNew;


	for (int i = 0; i < 100 - 1; i++)
	{
		dcDt[i] = dcDt[i + 1];
	}

	if (cNew > dNew)
	{
		dcDt[99] = cNew - dNew;
		std::cout << "c-d: Dt = " << dcDt[99] << "\n";
	}
	else
	{
		dcDt[99] = dNew - cNew;
		std::cout << "c-d: Dt = " << dcDt[99] << "\n";
	}


	avg = 0;
	for (int i = 0; i < 100; i++)
	{
		avg += dcDt[i] / 100;
	}

	if (avg > maxavg)maxavg = avg;
	std::cout << "avg: dcDt = " << avg << " max = " << maxavg << "\n";

}

void colorFrameProcess(VideoFrameRef colorFrame)
{
	Mat imgTmp = Mat::zeros(cvSize(640, 480), CV_8UC3);
	const RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*) colorFrame.getData();
	imgTmp.create(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3);
	memcpy(imgTmp.data, imageBuffer, 3 * colorFrame.getHeight()*colorFrame.getWidth()*sizeof(uint8_t));
	cv::cvtColor(imgTmp, imgTmp, CV_BGR2RGB); //this will put colors right
	colorDetection(&imgTmp);

	//std::cout << "relX = " << relX << " relY = " << relY << "\n";

	imshow("color", imgTmp);
}

void depthFrameProcess(VideoFrameRef depthFrame)
{

	DepthPixel* pixelAtTheColor[5];

	pixelAtTheColor[0] = (DepthPixel*)((char*)depthFrame.getData() + ((relY)* depthFrame.getStrideInBytes())) + relX - 1;
	pixelAtTheColor[1] = (DepthPixel*)((char*)depthFrame.getData() + (relY* depthFrame.getStrideInBytes())) + relX;
	pixelAtTheColor[2] = (DepthPixel*)((char*)depthFrame.getData() + ((relY)* depthFrame.getStrideInBytes())) + relX + 1;
	pixelAtTheColor[3] = (DepthPixel*)((char*)depthFrame.getData() + ((relY - 1)* depthFrame.getStrideInBytes())) + relX;
	pixelAtTheColor[4] = (DepthPixel*)((char*)depthFrame.getData() + ((relY + 1)* depthFrame.getStrideInBytes())) + relX;

	//std::cout << "relX= " << relX << std::endl;



	if ((float)(*pixelAtTheColor[1]) != 0)
	{
		//CoordinateConverter::convertDepthToWorld(depthSensor, (float)(relX), (float)(relY), (float)(*pixelAtTheColor[0]), &actX, &actY, &actZ);
		CoordinateConverter::convertDepthToWorld(depthSensor, (float)(relX - 1), (float)(relY), (float)(*pixelAtTheColor[0]), &actX, &actY, &actZArr[0]);
		CoordinateConverter::convertDepthToWorld(depthSensor, (float)(relX), (float)(relY), (float)(*pixelAtTheColor[0]), &actX, &actY, &actZArr[1]);
		CoordinateConverter::convertDepthToWorld(depthSensor, (float)(relX + 1), (float)(relY), (float)(*pixelAtTheColor[0]), &actX, &actY, &actZArr[2]);
		CoordinateConverter::convertDepthToWorld(depthSensor, (float)(relX), (float)(relY - 1), (float)(*pixelAtTheColor[0]), &actX, &actY, &actZArr[3]);
		CoordinateConverter::convertDepthToWorld(depthSensor, (float)(relX), (float)(relY + 1), (float)(*pixelAtTheColor[0]), &actX, &actY, &actZArr[4]);

		//std::cout << "relX = " << relX << " relY = " << relY << " relZ = " << *centerPixel << "\n";

		actZ = actZArr[0] + actZArr[1] + actZArr[2] + actZArr[3] + actZArr[4];
		actZ /= 5;


		int a = 0.96;

		prevActX = a*prevActX + (1 - a)*actX;
		prevActY = a*prevActY + (1 - a)*actY;
		prevActZ = a*prevActZ + (1 - a)*actZ;

		sendCoordinateMsg();
	}


	// window setting and show 
	depthMonitor(depthFrame);

}


void sendCoordinateMsg()
{
	int val = 0;

	if (prevActZ >= 0)
	{
		val = (int)(prevActZ + 0.5);
		if (val < 10)  messageX = "l1000" + std::to_string(val);
		else if (val <100)  messageX = "l100" + std::to_string(val);
		else if (val<1000)  messageX = "l10" + std::to_string(val);
		else if (val<10000) messageX = "l1" + std::to_string(val);
	}
	else
	{
		val = -1 * (int)(prevActZ - 0.5);
		if (val < 10)  messageX = "l2000" + std::to_string(val);
		else if (val <100)  messageX = "l200" + std::to_string(val);
		else if (val<1000)  messageX = "l20" + std::to_string(val);
		else if (val<10000) messageX = "l2" + std::to_string(val);
	}

	if (prevActX >= 0)
	{
		val = (int)(prevActX + 0.5);
		if (val < 10)  messageY = "l3000" + std::to_string(val);
		else if (val <100)  messageY = "l300" + std::to_string(val);
		else if (val<1000)  messageY = "l30" + std::to_string(val);
		else if (val<10000) messageY = "l3" + std::to_string(val);
	}
	else
	{
		val = -1 * (int)(prevActX - 0.5);
		if (val < 10)  messageY = "l4000" + std::to_string(val);
		else if (val <100)  messageY = "l400" + std::to_string(val);
		else if (val<1000)  messageY = "l40" + std::to_string(val);
		else if (val<10000) messageY = "l4" + std::to_string(val);
	}


	if (prevActY >= 0)
	{
		val = (int)(prevActY + 0.5);
		if (val < 10)  messageZ = "l5000" + std::to_string(val);
		else if (val <100)  messageZ = "l500" + std::to_string(val);
		else if (val<1000)  messageZ = "l50" + std::to_string(val);
		else if (val<10000) messageZ = "l5" + std::to_string(val);
	}
	else
	{
		val = -1 * (int)(prevActY - 0.5);
		if (val < 10)  messageZ = "l6000" + std::to_string(val);
		else if (val <100)  messageZ = "l600" + std::to_string(val);
		else if (val<1000)  messageZ = "l60" + std::to_string(val);
		else if (val<10000) messageZ = "l6" + std::to_string(val);
	}
	serialComm.sendString(messageX);
	serialComm.sendString(messageY);
	serialComm.sendString(messageZ);

	std::cout << std::fixed;
	std::cout << std::setprecision(1);
	std::cout << std::setw(10) << "actX = " << prevActX << std::setw(10) << "msg = " << messageX << endl
		<< std::setw(10) << "actY = " << prevActY << std::setw(10) << "msg = " << messageY << endl
		<< std::setw(10) << "actZ = " << prevActZ << std::setw(10) << "msg = " << messageZ << endl
		<< std::setw(10) << " in milli meters \n";


}

void depthMonitor(VideoFrameRef newFrame)
{
	if (status == STATUS_OK && newFrame.isValid())
	{
		// Clear the OpenGL buffers
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Setup the OpenGL viewpoint
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, window_w, window_h, 0, -1.0, 1.0);

		// UPDATING TEXTURE (DEPTH 1MM TO RGB888)
		unsigned short maxDepth = depthSensor.getMinPixelValue();
		unsigned short minDepth = depthSensor.getMaxPixelValue();
		for (int y = 0; y < newFrame.getHeight(); ++y)
		{
			DepthPixel* depthCell = (DepthPixel*)((char*)newFrame.getData() + (y * newFrame.getStrideInBytes()));
			for (int x = 0; x < newFrame.getWidth(); ++x, ++depthCell)
			{
				if (maxDepth < *depthCell)
				{
					maxDepth = *depthCell;
				}
				if (*depthCell != 0 && minDepth > *depthCell)
				{
					minDepth = *depthCell;
				}
			}
		}

		int depthHistogram[65536];
		int numberOfPoints = 0;
		if (histogram_enable)
		{
			memset(depthHistogram, 0, sizeof(depthHistogram));
			for (int y = 0; y < newFrame.getHeight(); ++y)
			{
				DepthPixel* depthCell = (DepthPixel*)((char*)newFrame.getData() + (y * newFrame.getStrideInBytes()));
				for (int x = 0; x < newFrame.getWidth(); ++x, ++depthCell)
				{
					if (*depthCell != 0)
					{
						depthHistogram[*depthCell]++;
						numberOfPoints++;
					}
				}
			}

			for (int nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
			{
				depthHistogram[nIndex] += depthHistogram[nIndex - 1];
			}
		}



		double resizeFactor = glMin((window_w / (double)newFrame.getWidth()), (window_h / (double)newFrame.getHeight()));
		unsigned int texture_x = (unsigned int)(window_w - (resizeFactor * newFrame.getWidth())) / 2;
		unsigned int texture_y = (unsigned int)(window_h - (resizeFactor * newFrame.getHeight())) / 2;

		for (unsigned int y = 0; y < (window_h - 2 * texture_y); ++y)
		{
			OniRGB888Pixel* texturePixel = gl_texture + ((y + texture_y) * window_w) + texture_x;
			DepthPixel lastPixel = 0;
			for (unsigned int x = 0; x < (window_w - 2 * texture_x); ++x, ++texturePixel)
			{
				DepthPixel* streamPixel = (DepthPixel*)((char*)newFrame.getData() + ((int)(y / resizeFactor) * newFrame.getStrideInBytes())) + (int)(x / resizeFactor);

				if (*streamPixel != 0)
				{
					lastPixel = *streamPixel;
				}
				else if (!blackfill_enable)
				{
					lastPixel = 0;
				}
				if (lastPixel != 0)
				{
					char depthValue = ((float)lastPixel / maxDepth) * 255;
					if (color_enable)
					{
						float colorPaletteFactor = (float)1024 / maxDepth;
						int colorCode = (lastPixel - minDepth) *  colorPaletteFactor;
						texturePixel->b = ((colorCode > 0 && colorCode < 512) ? abs(colorCode - 256) : 255);
						texturePixel->g = ((colorCode > 128 && colorCode < 640) ? abs(colorCode - 384) : 255);
						texturePixel->r = ((colorCode > 512 && colorCode < 1024) ? abs(colorCode - 768) : 255);
					}
					else
					{
						if (histogram_enable)
						{
							depthValue = ((float)depthHistogram[lastPixel] / numberOfPoints) * 255;
						}
						texturePixel->b = 255 - depthValue;
						texturePixel->g = 255 - depthValue;
						texturePixel->r = 255 - depthValue;
					}

				}
				else
				{
					texturePixel->b = lastPixel;
					texturePixel->g = lastPixel;
					texturePixel->r = lastPixel;
				}
			}
		}


		// Create the OpenGL texture map
		glTexParameteri(GL_TEXTURE_2D, 0x8191, GL_TRUE); // 0x8191 = GL_GENERATE_MIPMAP
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, window_w, window_h, 0, GL_RGB, GL_UNSIGNED_BYTE, gl_texture);

		glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glTexCoord2f(0.0f, 1.0f);
		glVertex3f(0.0f, (float)window_h, 0.0f);
		glTexCoord2f(1.0f, 1.0f);
		glVertex3f((float)window_w, (float)window_h, 0.0f);
		glTexCoord2f(1.0f, 0.0f);
		glVertex3f((float)window_w, 0.0f, 0.0f);
		glEnd();

		glutSwapBuffers();
	}
}

double glMin(double a, double b) {
	return (((a) < (b)) ? (a) : (b));
}

void colorDetection(Mat *img)
{

	//imshow("Original", *img);
	Mat imgHSV;

	Mat imgThresholded;

	cvtColor(*img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

										   //Threshold the image
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	imshow("Thresholded Image", imgThresholded); //show the thresholded image

	Moments oMoments = moments(imgThresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if (dArea > 10000)
	{
		//calculate the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;

		//printf("iLastX = %d, iLastY = %d, posx = %d, posY = %d\n", iLastX, iLastY, posX, posY);
		if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
		{
			//Draw a red line from the previous point to the current point
			//line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
			int r = (int)(sqrt((dArea * 4 / 3.14))*0.5);
			imgLines = Mat::zeros(cvSize(640, 480), CV_8UC3);
			circle(imgLines, Point(posX, posY), r / 13, Scalar(0, 255, 255), 2);

			//vertical lines
			line(imgLines, Point(imgLines.size().width / 2, imgLines.size().height / 2 - 20), Point(imgLines.size().width / 2, imgLines.size().height / 2 - 10), Scalar(255, 255, 255), 3);
			line(imgLines, Point(imgLines.size().width / 2, imgLines.size().height / 2 + 10), Point(imgLines.size().width / 2, imgLines.size().height / 2 + 20), Scalar(255, 255, 255), 3);
			// horizontal lines
			line(imgLines, Point(imgLines.size().width / 2 - 20, imgLines.size().height / 2), Point(imgLines.size().width / 2 - 10, imgLines.size().height / 2), Scalar(255, 255, 255), 3);
			line(imgLines, Point(imgLines.size().width / 2 + 20, imgLines.size().height / 2), Point(imgLines.size().width / 2 + 10, imgLines.size().height / 2), Scalar(255, 255, 255), 3);
		}

		relX = posX;
		relY = posY;

		//iLastX = posX;
		//iLastY = posY;
		//printf("---iLastX = %d, iLastY = %d, posx = %d, posY = %d\n", iLastX, iLastY, posX, posY);
	}



	*img = *img + imgLines;

}

