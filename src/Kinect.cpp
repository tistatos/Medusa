/**
 * @file Kinect.cpp
 *    description here
 * @author Erik Sandrén
 * @date  2015-05-12
 */

#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string.h>
#include "Kinect.h"
#include <pcl/common/transforms.h>

/**
 * @brief Default constructor

 * @param ctx the context for the kinect
 * @param index index of kinect (not used yet)
 */
Kinect::Kinect(freenect_context* ctx, int index): Freenect::FreenectDevice(ctx,index),
  mNewRgbFrame(false), mNewDepthFrame(false),mCameraCalibration(10, 640, 480, 7,5)
{

  mPosition = Eigen::Matrix4f::Identity();
  mBufferVideo = new uint8_t[freenect_find_video_mode(FREENECT_RESOLUTION_HIGH, FREENECT_VIDEO_RGB).bytes];
  mBufferDepth = new uint16_t[freenect_find_depth_mode(FREENECT_RESOLUTION_HIGH, FREENECT_DEPTH_REGISTERED).bytes/2];
  setFlag(FREENECT_AUTO_EXPOSURE, true);
  setFlag(FREENECT_AUTO_WHITE_BALANCE, true);
  setDepthFormat(FREENECT_DEPTH_REGISTERED);
  setVideoFormat(FREENECT_VIDEO_RGB);

  mCloud.width = 640;
  mCloud.height = 480;
  mCloud.points.resize(640*480);

  mCalibrated = false;

  mIndex = index;
}

/**
 * @brief default destructor
 */
Kinect::~Kinect()
{
  delete[] mBufferDepth;
  delete[] mBufferVideo;
}

/**
 * @brief Callback for video - DO NOT CALL EXPLICITLY!
 * @param video video frame data
 * @param timestamp when frame was taken
 */
void Kinect::VideoCallback(void *video, uint32_t timestamp)
{
  Mutex::ScopedLock lock(mRgbMutex);
  uint8_t* rgb = static_cast<uint8_t*>(video);
  memcpy(mBufferVideo, rgb, getVideoBufferSize());

  mNewRgbFrame = true;
}

/**
 * @brief Callback for depth - DO NOT CALL EXPLICITLY!
 * @param _depth depth frame data
 * @param timestamp when frame was taken
 */
void Kinect::DepthCallback(void *_depth, uint32_t timestamp)
{
  Mutex::ScopedLock lock(mDepthMutex);
  uint16_t* depth = static_cast<uint16_t*>(_depth);
  memcpy(mBufferDepth, depth, getDepthBufferSize()/2);

  float f = 595000.f;
  for (int i = 0; i < 640*480; ++i)
  {
    mCloud.points[i].x = (i%640 - (640-1)/2.f) * depth[i] / f;  // X = (x - cx) * d / fx
    mCloud.points[i].y = (i/640 - (480-1)/2.f) * depth[i] / f;  // Y = (y - cy) * d / fy
    mCloud.points[i].z = depth[i]/1000.f; // Z = d
  }
  mNewDepthFrame = true;
}

/**
 * @brief get latest video frame
 * @param frame pointer for returning data
 * @return true if new data was found, else false
 */
bool Kinect::getVideoFrame(VIDEO_IMAGE &image)
{
  if(!mNewRgbFrame)
    return false;
  Mutex::ScopedLock lock(mRgbMutex);

  for(int h = 0; h < 640*480*3; h+=3)
  {
    int y = h/(640*3);
    int x = (h/3)%640;
    image[y][x] = png::rgb_pixel(mBufferVideo[h], mBufferVideo[h+1], mBufferVideo[h+2]);
  }

  mNewRgbFrame = false;

  return true;
}

/**
 * @brief get latest depth frame
 * @param frame pointer for returning data
 * @return true if new data was found, else false
 */
bool Kinect::getDepthFrame(uint16_t **frame)
{

  if(!mNewDepthFrame)
    return false;

  Mutex::ScopedLock lock(mDepthMutex);
  *frame = new uint16_t[getDepthBufferSize()/2];

  memcpy(*frame, mBufferVideo, getDepthBufferSize()/2);

  mNewDepthFrame = false;

  return true;

}

/**
 * @brief get point cloud from latest frame, will also apply transformation on cloud
 * @todo only position at the moment, no orientation
 * @return the latest pointcloud
 */
pcl::PointCloud<pcl::PointXYZ> Kinect::getPointCloud()
{
  Mutex::ScopedLock lock(mDepthMutex);
  Eigen::Matrix4f transform =  mPosition.inverse();

  // Eigen::Matrix4f transform =  Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud(mCloud, *transformed_cloud, transform);
  return *transformed_cloud;
}

/**
 * @brief save current point cloud to a file
 * @param filename file name to save to
 */
void Kinect::savePointCloud(std::string filename)
{
  Mutex::ScopedLock lock(mDepthMutex);
  std::cout << mCloud.points.size() << std::endl;
  pcl::io::savePCDFileASCII (filename, mCloud);
}

/**
 * @brief check if new data from video is available
 * @return true if the new frames hasn't been accessed before
 */
bool Kinect::getVideoStatus()
{
  return mNewRgbFrame;
}


/**
 * @brief check if new data from depth is available
 * @return true if the new frames hasn't been accessed before
 */
bool Kinect::getDepthStatus()
{
  return mNewDepthFrame;
}

/**
 * @brief Set position of kinect
 * @param newPosition new position from calibration as matrix
 */
void Kinect::setPosition(cv::Mat newPosition)
{
  mPosition(0,0) = newPosition.at<double>(0,0);
  mPosition(1,0) = newPosition.at<double>(1,0);
  mPosition(2,0) = newPosition.at<double>(2,0);

  mPosition(0,1) = newPosition.at<double>(0,1);
  mPosition(1,1) = newPosition.at<double>(1,1);
  mPosition(2,1) = newPosition.at<double>(2,1);

  mPosition(0,2) = newPosition.at<double>(0,2);
  mPosition(1,2) = newPosition.at<double>(1,2);
  mPosition(2,2) = newPosition.at<double>(2,2);

  mPosition(0,3) = newPosition.at<double>(0,3)/1000.0f;
  mPosition(1,3) = newPosition.at<double>(1,3)/1000.0f;
  mPosition(2,3) = newPosition.at<double>(2,3)/1000.0f;

}

/**
 * @brief get transform of kinect
 * @return matrix of kinect
 */
Eigen::Matrix4f Kinect::getPosition()
{
  return mPosition;
}

/**
 * @brief calibrate kinect
 * @details takes on image with the kinect
 * @todo  increase this value for better dist. coeffectients and intrinsic values
 */
void Kinect::calibrate()
{
  if(mCameraCalibration.processedImages() == 10 && mCalibrated)
    return;

  std::cout << "trying to grab image" << std::endl;
  VIDEO_IMAGE image(640,480);
  getVideoFrame(image);
  mCameraCalibration.processImage(image);

  if(mCameraCalibration.processedImages() == 10 && !mCalibrated)
  {
    float resid = mCameraCalibration.calibrate();
    std::cout << "Residual: "<< resid << std::endl;
    mCalibrated = true;
  }
}

bool Kinect::setExtrinsic()
{
    VIDEO_IMAGE image(640,480);
    getVideoFrame(image);
    cv::Mat viewMatrix = mCameraCalibration.getCameraExtrinsic(image);
    if(countNonZero(viewMatrix) == 0)
      return false;
    setPosition(viewMatrix);
    std::cout << getPosition() << std::endl;
    return true;
}

bool Kinect::readCalibrationData()
{
  char filename[64];
  sprintf(filename, "calibration_data_%i.yaml", this->mIndex);
  std::ifstream f(filename);
  if(!f.good())
  {
    f.close();
    return false;
  }

  cv::FileStorage fs(filename, cv::FileStorage::READ);
  cv::Mat intrin, dist;
  fs["intrinsic"] >> intrin;
  fs["distortion"] >> dist;

  mCameraCalibration.setCalibrationData(intrin, dist);
  return true;

}

void Kinect::writeCalibrationData()
{
  char filename[64];
  sprintf(filename, "calibration_data_%i.yaml", this->mIndex);
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  fs << "intrinsic" << mCameraCalibration.getCameraIntinsic();
  fs << "distortion" << mCameraCalibration.getCameraDistCoeff();
}
