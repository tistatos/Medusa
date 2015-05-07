/**
 * @file calibration.h
 * @author Erik Sandrén
 * @date 2015-04-29
 */
#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include "libfreenect/libfreenect_sync.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <cmath>

#include <unistd.h>

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <opencv2/core/core.hpp>
#include <png++/png.hpp>

typedef png::image< png::rgb_pixel > VIDEO_IMAGE;

/**
 * @brief Class for camera calibration
 * @details both extrinsic and intrinsic
 *
 */
class Calibration
{
public:
  Calibration(int nPictures, int width, int height, int boardX, int boardY);
  int processImage(VIDEO_IMAGE frame);
  double calibrate();
  void clear();
  cv::Mat getCameraDistCoeff(){ return mDistCoeffs; }
  cv::Mat getCameraIntinsic() { return mCameraMatrix; }
  cv::Mat getCameraExtrinsic(VIDEO_IMAGE frame);
  bool calibrated() { return mCalibrated; }
  int processedImages() { return mProcessedImages; }
  static cv::Mat fromPNGtoMat(VIDEO_IMAGE);
private:
  std::vector<std::vector<cv::Point3f> > getObjectPoints();
  std::vector<std::vector<cv::Point2f> > mImagePoints; /// Points collected from images

  cv::Size mImageSize; /// Size of images used in calibration
  cv::Size mBoardSize; /// Size of chessboard used, value is interior corners
  float mGridSize; /// Size of each grid in the chessboard, given in mm

  int mNumberOfPictures; /// Number of images used in the calibration
  int mProcessedImages; /// Number of images that has been processed

  cv::Mat mCameraMatrix; /// Internal camera intrinsics
  cv::Mat mDistCoeffs; /// Internal camera distortion coefficients

  bool mCalibrated; /// is the camera calibrated?;
};



#endif
