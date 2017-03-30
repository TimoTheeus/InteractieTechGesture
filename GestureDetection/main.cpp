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
//amount f bins
int bins = 25;
int findBiggestContour(vector<vector<Point> >);

int main()
{
  // A class with helper functions
  Helper helper;
  
  // A class with for video input/output. In this case output to "output.avi" and input from device 0 (webcam)
  Video video("output.avi", 0);

  // Initialize the input (webcam)
  bool is_open_input = video.initializeInput();
  // Assert that it's really open
  CV_Assert(is_open_input);
  // Get the webcam caputure device
  auto video_capture = video.getCaptureDevice();
  
  int key = -1;
 /* 
 To initialize the video output, we need to give it the size(WxH) of the frames we will write to it.
 This size cannot change after inintializing. */

  bool is_open_output = video.initializeOutput(frame.size());
  CV_Assert(is_open_output);
  auto video_writer = video.getWriterDevice();
  while (key != 27)
  {
	  // Request an image from the webcam
	  video_capture >> frame;
	//convert to hsv
	  cvtColor(frame, hsv, COLOR_BGR2HSV);
	  Scalar min = (0, 10, 60);
	  Scalar max = (50, 0.68, 255);
	  inRange(hsv, Scalar(0, 10, 60), Scalar(20, 150, 255), skin);
	  imshow("skin", skin);
	  vector<vector<Point> > contours;
	  vector<Vec4i> hierarchy;
	  findContours(skin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	  int s = findBiggestContour(contours);

	  Mat drawing = Mat::zeros(frame.size(), CV_8UC1);
	  for (int i = 0; i < contours.size(); i++) {
		  drawContours(drawing, contours, i, Scalar(255), -1, 8, hierarchy, 0, Point());
	  }

	  imshow("drw", drawing);
	  video_writer << frame;
	  // Show the frame image
	  imshow(WEBCAM_WINDOW, frame);
	  // Get the keyboard input and wait 10ms to allow timely writing
	  key = waitKey(10);
  }

  // Release the video_writer (finish writing)
  video_writer.release();
  // Remove all open windows
  destroyAllWindows(); 
}

int findBiggestContour(vector<vector<Point> > contours) {
	int indexOfBiggestContour = -1;
	int sizeOfBiggestContour = 0;
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > sizeOfBiggestContour) {
			sizeOfBiggestContour = contours[i].size();
			indexOfBiggestContour = i;
		}
	}
	return indexOfBiggestContour;
}