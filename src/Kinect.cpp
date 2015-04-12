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

/**
 * @brief Default constructor

 * @param ctx the context for the kinect
 * @param index index of kinect (not used yet)
 */
Kinect::Kinect(freenect_context* ctx, int index): Freenect::FreenectDevice(ctx,index),
  mNewRgbFrame(false), mNewDepthFrame(false)
{

  mBufferVideo = new uint8_t[freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes];
  mBufferDepth = new uint16_t[freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes/2];

  setDepthFormat(FREENECT_DEPTH_REGISTERED);
  setVideoFormat(FREENECT_VIDEO_RGB);

  mCloud.width = 640;
  mCloud.height = 480;
  mCloud.points.resize(640*480);

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
bool Kinect::getVideoFrame(uint8_t **frame)
{
  if(!mNewRgbFrame)
    return false;
  Mutex::ScopedLock lock(mRgbMutex);
  *frame = new uint8_t[getVideoBufferSize()];

  memcpy(*frame, mBufferVideo, getVideoBufferSize());

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
 * @brief get point cloud from latest frame
 * @return the latest pointcloud
 */
pcl::PointCloud<pcl::PointXYZ> Kinect::getPointCloud()
{
  Mutex::ScopedLock lock(mDepthMutex);
  return mCloud;
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