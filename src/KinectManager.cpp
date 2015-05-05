/**
 * @file KinectManager.cpp
 * @author Erik Sandrén
 * @date  2015-05-12
 * @brief  kinect manager
 */

#include "KinectManager.h"

/**
 * @brief default constructor
 */
KinectManager::KinectManager()
{
#ifdef DEBUG
    printDebugMessage("Debugging on");
    freenect_set_log_level(m_ctx, FREENECT_LOG_DEBUG);
#else
    freenect_set_log_level(m_ctx, FREENECT_LOG_ERROR);
#endif
    mInitialized = true;
    mDevicesCalibrated = false;
}


/**
 * @brief get the number of devices found connected to system
 *
 * @return returns number of kinects
 */
int KinectManager::getDeviceCount()
{
  if(mInitialized)
    return deviceCount();
  else
    return -1;
}

/**
 * @brief connect to all devices found on system
 */
void KinectManager::connectToDevices()
{
  for (int i = 0; i < deviceCount(); ++i)
  {
    Kinect* device = &this->createDevice<Kinect>(i);
    mDevices.push_back(device);
  }
}

/**
 * @brief Get the number of devices connected AND added to the mangers
 *
 * @return number of kinects managed by manager
 */
int KinectManager::getConnectedDeviceCount()
{
  return mDevices.size();
}

/**
 * @brief destructor
 */
KinectManager::~KinectManager()
{
  mDevices.clear();
}

/**
 * @brief start depth callback for all kinects
 */
void KinectManager::startDepth()
{
  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->startDepth();
  }
}

/**
 * @brief start video callback for all kinects
 */
void KinectManager::startVideo()
{
  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->startVideo();
  }
}

/**
 * @brief stop depth callback for all kinects
 */
void KinectManager::stopDepth()
{
  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->stopDepth();
  }
}

/**
 * @brief stop video callback for all kinects
 */
void KinectManager::stopVideo()
{
  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->stopVideo();
  }
}

/**
 * @brief get depth data from kinects
 * @deprecated no need to have raw depth data
 * @param index index of managed kinect
 * @param frame unitialized pointer to store data
 *
 * @return true if there is a new frame to get from kinect
 */
bool KinectManager::getDepth(int index, uint16_t **frame)
{
  return false;
  // return mDevices[index]->getDepthFrame(frame);
}

bool KinectManager::getDepthStatus()
{
  if(getConnectedDeviceCount() == 0)
    return false;
  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    if(!mDevices[i]->getDepthStatus())
      return false;
  }
  return true;
}

/**
 * @brief get video data from kinects
 *
 * @param index index of managed kinect
 * @param frame unitialized pointer to store data
 *
 * @return true if there is a new frame to get from kinect
 */
bool KinectManager::getVideo(int index, VIDEO_IMAGE &frame)
{
  return mDevices[index]->getVideoFrame(frame);
}

bool KinectManager::getVideoStatus()
{
  if(getConnectedDeviceCount() == 0)
    return false;

  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    if(!mDevices[i]->getVideoStatus())
      return false;
  }
  return true;
}


/**
 * @brief return instance of managed kinect
 *
 * @param index index of managed kinect
 * @return the kinect
 */
Kinect* KinectManager::getDevice(int index)
{
  return mDevices[index];
}


void KinectManager::calibratePosition()
{
  //make calibration logic here
  startVideo();
  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->calibrate();
  }
  stopVideo();
  mDevicesCalibrated = true;
}
