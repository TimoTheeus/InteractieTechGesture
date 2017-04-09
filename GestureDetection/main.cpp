#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
#include <utility>
#include <vector>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include "Helper.h"
#include "Video.h"
#include "SerialClass.h"

using namespace cv;
using namespace std;

//for YCrCb transformation
Mat YCC;
/*Used for skin color filtering*/
Mat skin;
/*Used for storing masks to apply to the original frame*/
Mat dest;
/*Used for washing area*/
Mat washingArea;
//global variables 
int bins = 25;
double pi = 3.141592;

/*for gesture detection*/
bool pistolSign = false;
bool peaceSign = false;
bool washingHands = false;
int fingerTips;
Vec3b skinColor = (0, 0, 0);
CascadeClassifier face_cascade;
Mat frame; // current frame
/*face recognition*/
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
int contourSizeThreshold = 80;
bool outputInitialised = false;
//Erosion and dilation variables
int erosion_elem = 0;
int erosion_size = 2;
int dilation_elem = 0;
int dilation_size = 5;
//Trackbar variables
int color_slider_max = 100;
int Y_slider_upper = 90;
int Cr_slider_upper = 13;
int Cb_slider_upper = 80;
int Y_slider_lower = 78;
int Cr_slider_lower = 10;
int Cb_slider_lower = 80;
int rectangle_slider_maxX;
int rectangle_slider_maxY;
int upperLeftX = 66;
int upperLeftY = 348;
int bottomRightX = 230;
int bottomRightY = 473;
int skinSizeThreshold = 60;
//function headers
void blurImage(Mat);
void Erosion(Mat);
void Dilation(Mat);
void setSkinColor(Mat frame);
void findAndDrawContours(Mat src);
float getAngle(Point, Point, Point);
void checkPistolSign(Point, Point, double);
void displayDetectedGestures();
void sendDetectedGestures(Serial*);
void resetGestureVariables();
void checkPeaceSign(Point, Point,double);
double getDistance(Point, Point);
int biggestContourSize(Mat);
void determineHandWashing();
void initialiseColorSliders();
void initialiseWashingAreaSliders();

int main()
{
	/*variable for recognising keypresses*/
	int key = -1;
	// Open a Window
	

	Serial* SP = new Serial("COM3");    
	if (SP->IsConnected())
		printf("We're connected");

	// A class with helper functions
	Helper helper;

	/*Load cascades for face recognision*/
	if (!face_cascade.load(face_cascade_name)) { printf("--(!)Error loading\n"); return -1; };

	/* A class with for video input/output. In this case output to "output.avi" and input from device 0 (webcam)*/
	Video videoOriginal("original_frame.avi", 0);
	/* Initialize the input (webcam)*/
	bool is_open_input = videoOriginal.initializeInput();
	// Assert that it's really open
	CV_Assert(is_open_input);
	// Get the webcam caputure device
	auto video_capture = videoOriginal.getCaptureDevice();
	
	VideoWriter video_writer;
	/* Create window and sliders for adjusting the acceptable difference in color from the to be extracted skin color*/
	initialiseColorSliders();
	/*The main loop*/
	while (key != 27)
	{

		resetGestureVariables();

		/* Request an image from the webcam */
		video_capture >> frame;

		if (!outputInitialised) {
			/*
			To initialize the video output, we need to give it the size(WxH) of the frames we will write to it.
			This size cannot change after inintializing. */
			bool is_open_output = videoOriginal.initializeOutput(frame.size());
			CV_Assert(is_open_output);
			video_writer = videoOriginal.getWriterDevice();
			outputInitialised = true;
			initialiseWashingAreaSliders();
		}
		/*Initialise/Clear the final mask*/
		dest = Mat::zeros(frame.size(), CV_8UC1);

		/* convert to YCrCb */
		cvtColor(frame, YCC, CV_BGR2YCrCb);
		/* set skin color value based on face skin color and remove the face from the frame with a black rectangle*/
		setSkinColor(frame);

		/*Different color values*/
		int Y = skinColor.val[0];
		int Cr = skinColor.val[1];
		int Cb = skinColor.val[2];

		/* Acceptable bottom and upper ranges for skin color */
		Scalar bottomRange = Scalar(Y - Y_slider_lower, Cr - Cr_slider_lower, Cb - Cb_slider_lower);
		Scalar upperRange = Scalar(Y + Y_slider_upper, Cr + Cr_slider_upper, Cb + Cb_slider_upper);

		/*Filter the YCrCb color frame on skin color*/
		inRange(YCC, bottomRange, upperRange, skin);


		/*Dilate and then erode this filtered frame*/
		Erosion(skin);
		Dilation(skin);
		blurImage(skin);
		/*Show results of skin color filtering*/
		imshow("skin", skin);
		determineHandWashing();
		/*Draw the biggest contour, its convex hull and its convexity defects*/
		findAndDrawContours(skin);
		displayDetectedGestures();
		sendDetectedGestures(SP);
		/*Show the final mask with the convex hull and convexity defects drawn on it*/
		imshow("Final mask", dest);
		video_writer.write(frame);
		// Show the original frame image
		imshow(WEBCAM_WINDOW, frame);
		// Get the keyboard input and wait 10ms to allow timely writing
		key = waitKey(10);
	}

	// Release the video_writer (finish writing)
	video_writer.release();
	// Remove all open windows
	destroyAllWindows();
}

void initialiseColorSliders() {
	/* Create window and sliders for adjusting the acceptable difference in color from the to be extracted skin color*/
	namedWindow("Color_Adjuster", 1);
	createTrackbar("Y_lower", "Color_Adjuster", &Y_slider_lower, color_slider_max);
	createTrackbar("Cr_lower", "Color_Adjuster", &Cr_slider_lower, color_slider_max);
	createTrackbar("Cb_lower", "Color_Adjuster", &Cb_slider_lower, color_slider_max);
	createTrackbar("Y_upper", "Color_Adjuster", &Y_slider_upper, color_slider_max);
	createTrackbar("Cr_upper", "Color_Adjuster", &Cr_slider_upper, color_slider_max);
	createTrackbar("Cb_upper", "Color_Adjuster", &Cb_slider_upper, color_slider_max);
	createTrackbar("Cb_upper", "Color_Adjuster", &Cb_slider_upper, color_slider_max);
}
void initialiseWashingAreaSliders() {
	/* Create window and sliders for adjusting the washing area rectangle*/
	namedWindow("WashingArea_Adjuster", 1);
	rectangle_slider_maxX = frame.size().width;
	rectangle_slider_maxY = frame.size().height;
	createTrackbar("upperLeftX", "WashingArea_Adjuster", &upperLeftX, rectangle_slider_maxX);
	createTrackbar("upperLeftY", "WashingArea_Adjuster", &upperLeftY, rectangle_slider_maxY);
	createTrackbar("bottomRightX", "WashingArea_Adjuster", &bottomRightX, rectangle_slider_maxX);
	createTrackbar("upperRightY", "WashingArea_Adjuster", &bottomRightY, rectangle_slider_maxY);
	createTrackbar("skin_size_threshold", "WashingArea_Adjuster", &skinSizeThreshold, 200);
}
void determineHandWashing() {
	/*Initialise/Clear the mask used for handwashing recognition*/
	washingArea = Mat::zeros(frame.size(), CV_8UC1);
	rectangle(washingArea, Point(upperLeftX, upperLeftY), Point(bottomRightX, bottomRightY), cv::Scalar(255, 255, 255), -1, 8, 0);
	skin.copyTo(washingArea, washingArea);
	/*Draw rectangle to indicate the currently selected area*/
	rectangle(washingArea, Point(upperLeftX, upperLeftY), Point(bottomRightX, bottomRightY), cv::Scalar(255, 255, 255), 2, 8, 0);
	int skinSizeInWashingArea = biggestContourSize(washingArea);
	if (skinSizeInWashingArea > skinSizeThreshold)
	{
		Helper::putPrettyText("is washing hands", Point(320, 400), 1, washingArea);
		washingHands = true;
	}
	imshow("washing", washingArea);
}
void displayDetectedGestures() {
	if (pistolSign)
	{
		Helper::putPrettyText("Pistol Sign Detected", Point(10, 10), 1, dest);
	}
	else if (peaceSign) {
		Helper::putPrettyText("Peace Sign Detected", Point(10, 20), 1, dest);
	}
	else { Helper::putPrettyText("No Gesture Detected", Point(10, 10), 1, dest); }
}
void sendDetectedGestures(Serial* SP) {
	if (SP->IsConnected()) {
		if (pistolSign)
		{
			const char* oneSpray = new char[1];
			oneSpray = "1";
			SP->WriteData(oneSpray, 1);
		}
		else if (peaceSign) {
			const char* twoSprays = new char[1];
			twoSprays = "2";
			SP->WriteData(twoSprays, 1);
		}
		if (washingHands) {
			const char* iswashing = new char[1];
			iswashing = "w";
			SP->WriteData(iswashing, 1);
		}
	}
}
float getAngle(Point a, Point b, Point c) {
	Vec2f ab = Vec2f(a.x - b.x, a.y - b.y);
	Vec2f bc = Vec2f(c.x - b.x, c.y - b.y);
	float abLngth = sqrt(pow(ab[0], 2) + pow(ab[1], 2));
	float bcLngth = sqrt(pow(bc[0], 2) + pow(bc[1], 2));
	Vec2f nrmlAB = Vec2f(ab[0] / abLngth, ab[1] / abLngth);
	Vec2f nrmlBC = Vec2f(bc[0] / bcLngth, bc[1] / bcLngth);
	float dot = nrmlAB[0] * nrmlBC[0] + nrmlAB[1] * nrmlBC[1];
	float angle = (acos(dot) / pi) * 180;
	return angle;
}

void resetGestureVariables(){
	pistolSign = false;
	peaceSign = false;
	fingerTips = 0;
	washingHands = false;
}
/*Dilate an input */
void Dilation(Mat input)
{
	int dilation_type;

	/*Determine dilation type*/
	if (dilation_elem == 0) { dilation_type = MORPH_RECT; }
	else if (dilation_elem == 1) { dilation_type = MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));

	/* Apply the dilation operation*/
	dilate(input, input, element);
}
/*Erode an input*/
void Erosion(Mat input) {
	Mat eroded = Mat::zeros(frame.size(), CV_8UC1);
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	erode(input, input, element);
}

/*Blur an input */
void blurImage(Mat input) {
	Mat blurred = Mat::zeros(frame.size(), CV_8UC1);
	for (int i = 1; i < 7; i = i + 2)
	{
		medianBlur(input, input, i);
	}
}

int biggestContourSize(Mat src) {
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int biggestSize = 0;
	/*Find the contours*/
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/*Find the biggest contour and its index*/
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > biggestSize) {
			biggestSize = contours[i].size();
		}
	}
	return biggestSize;
}
/*Find and show the convex hull and convexity defects of the biggest contour */
void findAndDrawContours(Mat src)
{
	/*Variables used for getting/storing the contours*/
	int biggestContour = 0; int indexBiggest = 0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/*Variables used for getting/storing the convex hull*/
	Mat convexhull = Mat::zeros(src.size(), CV_8UC3);
	vector<vector<Point> >hull(1);

	/*Variables used for getting/storing the convexity defects*/
	vector<int> hullIndexes;
	std::vector<Vec4i> defects;

	/*Drawing of the biggest contour*/
	Mat drawing = Mat::zeros(frame.size(), CV_8UC1);

	/*Find the contours*/
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/*Find the biggest contour and its index*/
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > biggestContour) {
			biggestContour = contours[i].size();
			indexBiggest = i;
		}
	}
	/*If the contour is above a certain threshhold*/
	if (indexBiggest<contours.size() && contours[indexBiggest].size()>contourSizeThreshold) {
		/*Draw the biggest contour, fill it in and store it in drawing*/
		drawContours(drawing, contours, indexBiggest, Scalar(255), -1, 8, hierarchy, 0, Point());

		/*Apply the mask to the original frame and store it*/
		frame.copyTo(dest, drawing);

		/*Get the convex hull of the biggest contour and draw it*/
		convexHull(Mat(contours[indexBiggest]), hull[0], false);
		drawContours(dest, hull, 0, Scalar(255, 0, 255), 1, 8, hierarchy, 0, Point());

		/*Get the convexity defects of the convex hull*/
		convexHull(Mat(contours[indexBiggest]), hullIndexes, false);
		if (hullIndexes.size() > 3)
		{
			convexityDefects(contours[indexBiggest], hullIndexes, defects);
		}
		/* Get the center of the boundingbox of the hull*/
		Rect boundingBox = boundingRect(hull[0]);
		Point centerHull = Point(boundingBox.x + boundingBox.width*0.5, boundingBox.y + boundingBox.height*0.5);

		/*Get spacial extremes of the hull (uit slides)*/
		auto minimax_x = std::minmax_element(
			hull[0].begin(), hull[0].end(),
			[](const cv::Point &p1, const cv::Point &p2)
		{
			return p1.x < p2.x;
		});
		cv::Point left(minimax_x.first->x, minimax_x.first->y);
		cv::Point right(minimax_x.second->x, minimax_x.second->y);


		
		/*Draw the convexity defects*/
		const int arraySize = 2;
		Point points[2][arraySize];
		double angles[arraySize];
		for (int j = 0; j<defects.size(); j++)
		{
			Point p1 = contours[indexBiggest][defects[j][0]];
			Point p2 = contours[indexBiggest][defects[j][1]];
			Point p3 = contours[indexBiggest][defects[j][2]];
			line(dest, p1, p3, Scalar(0, 0, 255), 2);
			line(dest, p2, p3, Scalar(0, 0, 255), 2);

			/*see if it can be a finger top by calculating the angle with the other points, and its position relative to the center of the hull*/
			double angle = getAngle(p1, p3, p2);
			if (p1.y<centerHull.y && abs(angle) > 20 && angle < 120) {
				/*draw finger tip*/
				circle(dest, p1, 9, Scalar(0, 255, 0), 2);

				/*Store fingertips and their value for peace sign recognition*/
				if (fingerTips < 2) {
					points[0][fingerTips] = p1;
					points[1][fingerTips] = p2;
					angles[fingerTips] = abs(angle);
				}
				fingerTips++;
				/*Check for pistol sign and peace sign gesture*/
				checkPistolSign(p1, right, abs(angle));
			}
			//circle(dest, right, 8, Scalar(0, 0, 255), 2);
		}
		/*Check for the 2 fingertops if the neighbour is also a fingertop and then check for a peace sign*/
		if (fingerTips == 2) {
			Point a1 = points[0][0];
			Point a2 = points[1][0];
			Point b1 = points[0][1];
			Point b2 = points[1][1];
			if (!washingHands) {
				/*If they are neighbours*/
				if (getDistance(a1, b2) < 30)
				{
					/*check for a peace sign*/
					checkPeaceSign(b1, b2, angles[1]);
				}
				if (getDistance(a2, b1) < 50)
				{
					checkPeaceSign(a1, a2, angles[0]);
				}
			}
		}
	}
}

double getDistance(Point p1, Point p2) {
	double distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	return distance;
}
void checkPeaceSign(Point tipOne, Point tipTwo, double angle) {
	double distance = getDistance(tipOne, tipTwo);
	if (angle>35 && angle < 60) {
		peaceSign = true;
	}
}
void checkPistolSign(Point fingertip, Point extreme, double angle) {
	double distance = getDistance(fingertip, extreme);
	if (distance < 15 && angle>75 && angle<120)
	{
		pistolSign = true;
	}
}
/*Try to detect a face and extract a guideline skin color value from it*/
void setSkinColor(Mat frame)
{
	std::vector<Rect> faces;
	Mat grayframe;
	cvtColor(frame, grayframe, CV_BGR2GRAY);
	//-- Detect faces
	face_cascade.detectMultiScale(grayframe, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(120, 120));
	for (size_t i = 0; i < faces.size(); i++)
	{
		int width = faces[i].width;
		int height = faces[i].height;
		Point center(faces[i].x + width*0.5, faces[i].y + height*0.5);
		skinColor = YCC.at<Vec3b>(center);;
		//remove face from frame and YCrCb color frame by drawing a rectangle over it 
		rectangle(frame, center - Point(width*0.5, height*0.9), center + Point(width*0.5, height*0.9), cv::Scalar(0, 0, 0), -1, 8, 0);
		rectangle(YCC, center - Point(width*0.5, height*0.9), center + Point(width*0.5, height*0.9), cv::Scalar(0, 0, 0), -1, 8, 0);
	}
}