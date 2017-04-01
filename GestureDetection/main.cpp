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

using namespace cv;
using namespace std;

// An image is a matrix (cv::Mat) in OpenCV
Mat frame;
//for hsv transformation
Mat hsv;
//skincolor
Mat skin;
//global variables 
int bins = 25;
Vec3b skinColor = (10, 80, 150);
CascadeClassifier face_cascade;
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
//function headers
Mat returnBlurred(Mat);
Mat returnEroded(Mat, int);
void setSkinColor(Mat frame);
void findAndDrawContours(Mat src);

int main()
{
  // A class with helper functions
  Helper helper;
  //-- 1. Load the cascades
  if (!face_cascade.load(face_cascade_name)) { printf("--(!)Error loading\n"); return -1; };
  // A class with for video input/output. In this case output to "output.avi" and input from device 0 (webcam)
  Video video("output.avi", 0);
  // Initialize the input (webcam)
  bool is_open_input = video.initializeInput();
  // Assert that it's really open
  CV_Assert(is_open_input);
  // Get the webcam caputure device
  auto video_capture = video.getCaptureDevice();
  //keypress variable
  int key = -1;
 /* 
 To initialize the video output, we need to give it the size(WxH) of the frames we will write to it.
 This size cannot change after inintializing. */
  bool is_open_output = video.initializeOutput(frame.size());
  CV_Assert(is_open_output);
  auto video_writer = video.getWriterDevice();
  //The loop
  while (key != 27)
  {
	  // Request an image from the webcam
	  video_capture >> frame;
	  //convert to hsv
	  cvtColor(frame, hsv, COLOR_BGR2HSV);
	  //set skin color value based on face skin color
	  setSkinColor(frame);
	  int H = skinColor.val[0]; //hue
	  int S = skinColor.val[1]; //saturation
	  int V = skinColor.val[2]; //value
	  //The range of HSV colors detected as skin colors
	  Scalar bottomRange = Scalar(H-30, S-90, V-80);
	  Scalar upperRange = Scalar(H + 30, S + 90, V + 80);
	  //filter on skin color
	  inRange(hsv, bottomRange, upperRange, skin);
	  //Show results
	  imshow("skin", skin);
	  //Draw 3 biggest contours
	  findAndDrawContours(skin);
	  video_writer << frame;
	  // Show the original frame image
	  imshow(WEBCAM_WINDOW, frame);
	  // Get the keyboard input and wait 10ms to allow timely writing
	  key = waitKey(100);
  }

  // Release the video_writer (finish writing)
  video_writer.release();
  // Remove all open windows
  destroyAllWindows(); 
}

Mat returnEroded(Mat input,int erosion_size) {
	Mat eroded = Mat::zeros(frame.size(), CV_8UC1);
	Mat element = getStructuringElement(MORPH_RECT,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	erode(input, eroded, element);
	return eroded;
}
Mat returnBlurred(Mat input) {
	Mat blurred = Mat::zeros(frame.size(), CV_8UC1);
	for (int i = 1; i < 31; i = i + 2)
	{
		blur(input, blurred, Size(i, i), Point(-1, -1));
	}
	return blurred;
}

//Find and draw the 3 biggest contours, this removes a lot of noise and still allows a head and 2 hands to be drawn
void findAndDrawContours(Mat src)
{
	//size and index used to find 3 biggest contours (head + hands)
	int sizeFirst = 0; int indexFirst = 0;
	int sizeSecond = 0; int indexSecond = 0;
	int sizeThird = 0; int indexThird = 0;
	
	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	Mat drawing = Mat::zeros(frame.size(), CV_8UC1);
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > sizeFirst) {
			sizeThird = sizeSecond;
			sizeSecond = sizeFirst;
			sizeFirst = contours[i].size();
			indexThird = indexSecond;
			indexSecond = indexFirst;
			indexFirst = i;
		}
		else if (contours[i].size() > sizeSecond) {
			sizeThird = sizeSecond;
			sizeSecond = contours[i].size();
			indexThird = indexSecond;
			indexSecond = i;
		}
		else if (contours[i].size() > sizeThird)
		{
			sizeThird = contours[i].size();
			indexThird = i;
		}
	}
	drawContours(drawing, contours, indexThird, Scalar(255), -1, 8, hierarchy, 0, Point());
	drawContours(drawing, contours, indexSecond, Scalar(255), -1, 8, hierarchy, 0, Point());
	drawContours(drawing, contours, indexFirst, Scalar(255), -1, 8, hierarchy, 0, Point());
	Mat blurred = Mat::zeros(frame.size(), CV_8UC1);
	imshow("drw", drawing);
	blurred = returnBlurred(drawing);
	imshow("blur",blurred);
}
//set skin color to value detected in a face3
void setSkinColor(Mat frame)
{
	std::vector<Rect> faces;
	Mat frame_gray;
	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);
	//-- Detect faces
	face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

	for (size_t i = 0; i < faces.size(); i++)
	{
		int width = faces[i].width;
		int height = faces[i].height;
		Point center(faces[i].x + width*0.5, faces[i].y + height*0.5);
		ellipse(frame, center, Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
		skinColor = hsv.at<Vec3b>(center);;
	}
}