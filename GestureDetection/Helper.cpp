#include <numeric>
#include <opencv2/opencv.hpp>

#include "Helper.h"

using namespace cv;
using namespace std;

Helper::Helper(void)
{
  // Reserve some memory
  m_cvmat2str.reserve(19);

  // Emplace all string interpretations (R) into the vector with the correct type macro (L)
  m_cvmat2str.emplace_back(pair<int, string>(-1, (string) "CV_???"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_8U, (string) "CV_8U"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_8UC3, (string) "CV_8UC3"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_8S, (string) "CV_8S"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_16U, (string) "CV_16U"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_16S, (string) "CV_16S"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_32S, (string) "CV_32S"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_32F, (string) "CV_32F"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_32FC2, (string) "CV_32FC2"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_32FC3, (string) "CV_32FC3"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_32FC4, (string) "CV_32FC4"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_32FC(5), (string) "CV_32FC5"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_64F, (string) "CV_64F"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_64FC2, (string) "CV_64FC2"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_64FC3, (string) "CV_64FC3"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_64FC4, (string) "CV_64FC4"));
  m_cvmat2str.emplace_back(pair<int, string>(CV_64FC(5), (string) "CV_64FC5"));
}

Helper::~Helper(void)
{
}

const string Helper::showCVMatType(int type)
{
  // We use a lambda function predicate to find the mapping of the type integer to its string description
  // We catch the type variable to make it available in the lambda function
  // std::find_if return a const_iterator that points to its current content
  vector<pair<int, string>>::const_iterator type_iterator = 
    find_if(m_cvmat2str.begin(), m_cvmat2str.end(), [type](const pair<int, string> &str_type)
  {
    return str_type.first == type;
  });

  // If the iterator is not at the end of the vector, we've found the right interpretation
  if (type_iterator != m_cvmat2str.end())
  {
    // to get to the content of the const_iterator, we need to use the pointer (->)
    return type_iterator->second;
  }

  // Else we didn't find the right interpretation, so we return "CV_???"
  return m_cvmat2str.front().second;
}

void Helper::putPrettyText(const string &text, const Point &location, const double text_size, Mat &canvas)
{
	const int thickness1 = 2;
	// FONT_HERSHEY_PLAIN is the font type
	// Color_BLACK is the paint color
	// CV_AA means the text is anti-aliased
	putText(canvas, text, location, FONT_HERSHEY_PLAIN, text_size, Color_BLACK, thickness1, CV_AA);

	const int thickness2 = 1;
	putText(canvas, text, location, FONT_HERSHEY_PLAIN, text_size, Color_WHITE, thickness2, CV_AA);
}

void Helper::initRandomIntVector(const int size, const int offset)
{
	// Reset
	m_random_numbers.clear();
	// Allocate some space for some random numbers
	m_random_numbers.resize(size);
	// Generate a list start with: 0, 1, 2, 3, 4, 5, 6, ...
	iota(m_random_numbers.begin(), m_random_numbers.end(), offset);
	// Shuffle the list to make the order random (eg.): 5, 1, 9, 4, 6, 7, 0, ...  
	random_shuffle(m_random_numbers.begin(), m_random_numbers.end());
}
