#include "Video.h"

using namespace cv;

Video::Video(const std::string &output, const int input) :
  m_video_capture(nullptr),
  m_video_writer(nullptr),
  m_input(input),
  m_output(output),
  m_fps(30),
  m_fourcc(CV_FOURCC('M', 'P', 'E', 'G'))
{
}

Video::~Video()
{
  // Stop/close capture/write video
  closeInput();
  closeOutput();
}

bool Video::initializeInput()
{
  if (m_video_capture == nullptr)
    m_video_capture = std::make_shared<VideoCapture>();
  else
    return false; // already initialized

  m_video_capture->open(m_input);

  // Wait for the webcam to fire up (give 250ms)
  int timeout = 0;
  Mat dummy;
  while (dummy.empty() && timeout++ < 250)
  {
    *m_video_capture >> dummy;
    waitKey(1);
  }

  return m_video_capture->isOpened();
}

bool Video::initializeOutput(const Size &video_size)
{
  if (m_video_writer == nullptr)
    m_video_writer = std::make_shared<VideoWriter>();
  else
    return false; // already initialized

  m_video_writer->open(m_output, m_fourcc, m_fps, video_size);
  return m_video_writer->isOpened();
}