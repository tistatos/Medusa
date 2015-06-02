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

  startVideo();

  bool calibrated = false;
  time_t start = time(0);
  std::cout << "calibration" << std::endl;

  while(!calibrated)
  {
    double seconds_since_start = difftime( time(0), start);
    if(seconds_since_start < 1)
      continue;
    calibrated = true;
    for (int i = 0; i < getConnectedDeviceCount(); ++i)
    {
      mDevices[i]->calibrate();

      if(calibrated)
        calibrated = mDevices[i]->isCalibrated();
    }
    start = time(0);
  }
  stopVideo();
  mDevicesCalibrated = true;

  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->writeCalibrationData();
  }
}

void KinectManager::setOrigin()
{
  startVideo();
  bool positioned = false;
  while(!positioned) {
    positioned = true;
    for (int i = 0; i < getConnectedDeviceCount(); ++i)
    {
      bool devPositioned = mDevices[i]->setExtrinsic();

      if(positioned)
        positioned = devPositioned;
    }
  }
  stopVideo();

  for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    Texture::applyCameraPose(getDevice(i));
  }
}


bool KinectManager::loadCalibration()
{
  int kinectCount = getConnectedDeviceCount();
  DIR* pDir = opendir("./");
  if(pDir != NULL)
  {
    struct dirent *pDirent;
    while((pDirent = readdir(pDir)) != NULL)
    {
       int len = strlen (pDirent->d_name);
        if (len >= 4) {
            if (strcmp (".yaml", &(pDirent->d_name[len - 4])) == 0) {
                printf ("%s\n", pDirent->d_name);
            }
        }
    }
  }
  else
  {
    return false;
  }

  for (int i = 0; i < kinectCount; ++i)
  {
    if(!mDevices[i]->readCalibrationData())
      return false;
  }
  return true;
}


void KinectManager::setFilename(std::string filename)
{
   for (int i = 0; i < getConnectedDeviceCount(); ++i)
  {
    mDevices[i]->setFilename(filename);
  }
}