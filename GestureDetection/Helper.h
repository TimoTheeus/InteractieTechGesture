#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#define IMAGE_WINDOW "Image window"
#define WEBCAM_WINDOW "Webcam window"
#define TRACKBAR_NAME "Trackbar"

/*!
  A list of extensive color definitions, taken from:
  https://xkcd.com/color/rgb/
*/
const static cv::Scalar Color_BLACK = CV_RGB(0x00, 0x00, 0x00);
const static cv::Scalar Color_GRAY = CV_RGB(0x92, 0x95, 0x91);
const static cv::Scalar Color_WHITE = CV_RGB(0xFF, 0xFF, 0xFF);
const static cv::Scalar Color_RED = CV_RGB(0xE5, 0x00, 0x00);
const static cv::Scalar Color_RED_LIGHT = CV_RGB(0xFF, 0x47, 0x4C);
const static cv::Scalar Color_RED_FLAG = CV_RGB(0xAE, 0x1C, 0x28);
const static cv::Scalar Color_RED_DARK = CV_RGB(0x84, 0x00, 0x00);
const static cv::Scalar Color_GREEN = CV_RGB(0x15, 0xB0, 0x1A);
const static cv::Scalar Color_GREEN_BRIGHT = CV_RGB(0x01, 0xFF, 0x07);
const static cv::Scalar Color_GREEN_LIME = CV_RGB(0x89, 0xFE, 0x05);
const static cv::Scalar Color_GREEN_LIGHT = CV_RGB(0x96, 0xF9, 0x7B);
const static cv::Scalar Color_GREEN_FOREST = CV_RGB(0x06, 0x47, 0x0C);
const static cv::Scalar Color_GREEN_DARK = CV_RGB(0x03, 0x35, 0x00);
const static cv::Scalar Color_BLUE = CV_RGB(0x03, 0x43, 0xDF);
const static cv::Scalar Color_BLUE_LIGHT = CV_RGB(0x95, 0xD0, 0xFC);
const static cv::Scalar Color_BLUE_SKY = CV_RGB(0x75, 0xBB, 0xFD);
const static cv::Scalar Color_BLUE_FLAG = CV_RGB(0x21, 0x46, 0x8B);
const static cv::Scalar Color_BLUE_DARK = CV_RGB(0x00, 0x03, 0x5B);
const static cv::Scalar Color_PURPLE = CV_RGB(0x7E, 0x1E, 0x9C);
const static cv::Scalar Color_PURPLE_LIGHT = CV_RGB(0xBF, 0x77, 0xF6);
const static cv::Scalar Color_PURPLE_DARK = CV_RGB(0x35, 0x06, 0x3E);
const static cv::Scalar Color_PINK = CV_RGB(0xFF, 0x81, 0xC0);
const static cv::Scalar Color_YELLOW = CV_RGB(0xFF, 0xFF, 0x14);
const static cv::Scalar Color_ORANGE = CV_RGB(0xF9, 0x73, 0x06);
const static cv::Scalar Color_ORANGE_FLAG = CV_RGB(0xFF, 0x7F, 0x00);
const static cv::Scalar Color_BROWN = CV_RGB(0x65, 0x37, 0x00);
const static cv::Scalar Color_MAGENTA = CV_RGB(0xC2, 0x00, 0x78);
const static cv::Scalar Color_TEAL = CV_RGB(0x02, 0x93, 0x86);
const static cv::Scalar Color_TURQUOISE = CV_RGB(0x06, 0xC2, 0xAC);
const static cv::Scalar Color_VIOLET = CV_RGB(0x9A, 0x0E, 0xEA);
const static cv::Scalar Color_LAVENDER = CV_RGB(0xC7, 0x9F, 0xEF);
const static cv::Scalar Color_TAN = CV_RGB(0xD1, 0xB2, 0x6F);
const static cv::Scalar Color_CYAN = CV_RGB(0x00, 0xFF, 0xFF);
const static cv::Scalar Color_AQUA = CV_RGB(0x13, 0xEA, 0xC9);
const static cv::Scalar Color_MAUVE = CV_RGB(0xAE, 0x71, 0x81);
const static cv::Scalar Color_MAROON = CV_RGB(0x65, 0x00, 0x21);
const static cv::Scalar Color_OLIVE = CV_RGB(0x6E, 0x75, 0x0E);
const static cv::Scalar Color_COBALT = CV_RGB(0x1E, 0x48, 0x8F);
const static cv::Scalar Color_KHAKI = CV_RGB(0xAA, 0xA6, 0x62);

//!  A simple tutorial class
/*!
  This class contains some helper functions. It also shows the basics
  of passing variables by reference (&) to a function in C/C++, which you
  should always prefer for non-primitive variables over passing by 
  local copy or passing by memory pointer (*).
*/
class Helper
{
  //! A vector of pairs to connect numbers to matrix type names
  std::vector<std::pair<int, std::string>> m_cvmat2str;
  //! A vector of integers
  std::vector<int> m_random_numbers;

public:
  Helper(void);
  ~Helper(void);

  /*!
    This function converts a cv::Mat::type() to a readable string
  */
  /*!
  /param type An integer with the cv::Mat type as it is returned from: cv::Mat matrix.type()
  */
  const std::string showCVMatType(int type);

  /*!
    This static function draws a white text with a black outline on an image, 
    so it's always readable. It uses anti-aliasing to make it extra sweet!
  */
  /*!
  /param text A string constant argument with text
	/param location A location constant argument with a cv::Point for where to put the text
	/param canvas An image by reference to draw the text onto
  */
  static void putPrettyText(const std::string &text, const cv::Point &location, const double text_size, cv::Mat &canvas);
  
  /*!
    This initializes the Tutorial class member m_random_numbers. It fills the 
	vector with random numbers.
  */
  /*!
  /param size A constant integer that sets the amount of random numbers in the vector
	/param offset A constant integer that sets the lower boundary of the random numbers, which defaults to 0
  */
  void initRandomIntVector(const int size, const int offset = 0);

  //! Getter by reference for the random integer vector m_random_numbers
  const std::vector<int> &getRandomIntVector() const
  {
    return m_random_numbers;
  }

  //! Setter to put a new vector in place of class member m_random_numbers
  void setRandomIntVector(const std::vector<int> &random_int_vector)
  {
    m_random_numbers = random_int_vector;
  }
};

