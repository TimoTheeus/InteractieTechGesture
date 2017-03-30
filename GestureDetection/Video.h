#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

typedef std::shared_ptr<cv::VideoCapture> SVideoCapture;
typedef std::shared_ptr<cv::VideoWriter> SVideoWriter;

/*
This class handles video input (from the webcam) and output (to a video file).
It has a capture device and a writer device, accessible through getters and setters
*/
class Video
{
  SVideoCapture m_video_capture;
  SVideoWriter m_video_writer;

  const int m_input;
  const std::string m_output;

  int m_fourcc;
  int m_fps;

public:
  /*!
    This is the constructor, it requires an input number for 
    your webcam device and an output filename (if given without
    a path, the video file is written in your project root folder).

    The default input is 0
  */
  /*!
  /param output the video file to write to
  /param input the input device to get images from
  */
  Video(const std::string &output, const int input = 0);
	~Video();

  /*!
    Initialize the input device
  */
  /*!
  returns false if already initialized (use closeInput() first)
  */
  bool initializeInput();

  /*!
    Initialize the output file
  */
  /*!
  returns false if already initialized (use closeOutput() first)
  */
  bool initializeOutput(const cv::Size &video_size);

  /*!
    The input device
  */
  const cv::VideoCapture &getCaptureDevice()
  {
    return *m_video_capture;
  }

  /*!
    The output device
  */
  const cv::VideoWriter &getWriterDevice()
  {
    return *m_video_writer;
  }

  /*!
    The fourcc encoding parameter. Use setFourCC(..) before initializing the output device to change.
  */
  const int getFourCC() const
  {
    return m_fourcc;
  }

  /*!
  FOURCC decides the encoding of the video we will write.
  This should work if you have an MPEG encoder installed,
  other options:
  setFourCC(CV_FOURCC('M', 'P', 'E', 'G'));
  setFourCC(CV_FOURCC('D', 'I', 'V', 'X'));
  setFourCC(CV_FOURCC('X', 'V', 'I', 'D'));
  more exotic options are available, check:
  setFourCC(CV_FOURCC_PROMPT);
  Ultimately you can set fourcc to -1, this will cause a menu
  to pop-up at runtime with which you can choose your codec

  The default is CV_FOURCC('M', 'P', 'E', 'G')
  */
  /*!
  /param fourcc the fourcc code acquired from CV_FOURCC
  */
  void setFourCC(const int fourcc)
  {
    m_fourcc = fourcc;
  }

  /*!
    This is the amount of Frames Per Second that the output video
    is set to.
  */
  const int getFPS() const
  {
    return m_fps;
  }

  /*!
    Here you set the amount of Frames Per Second of the output video.
    Change it before initializing the device. The default is 30.
  */
  /*!
  /param fps the amount of frames per second
  */
  void setFPS(const int fps)
  {
    m_fps = fps;
  }

  /*!
    Close the input device.
  */
  void closeInput()
  {
    m_video_capture->release();
    m_video_capture = nullptr;
  }

  /*!
    Close the output device. Reinitializing will create a new video, overwriting the old one!
  */
  void closeOutput()
  {
    m_video_writer->release();
    m_video_writer = nullptr;
  }
};

