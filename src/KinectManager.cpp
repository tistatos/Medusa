/**
 * @File KinectManager.cpp
 *    description here
 * @autor Erik Sandrén
 * @date  DATE
 */

#include "KinectManager.h"

KinectManager::KinectManager()
{
#ifdef DEBUG
    printDebugMessage("Debugging on");
    freenect_set_log_level(m_ctx, FREENECT_LOG_DEBUG);
#else
    freenect_set_log_level(m_ctx, FREENECT_LOG_ERROR);
#endif
    mInitialized = true;
}


int KinectManager::getDeviceCount()
{
  if(mInitialized)
    return deviceCount();
  else
    return -1;
}

void KinectManager::connectToDevices()
{
  for (int i = 0; i < deviceCount(); ++i)
  {
    Kinect* device = &this->createDevice<Kinect>(i);
    mDevices.push_back(device);
  }
}

int KinectManager::getConnectedDeviceCount()
{
  return mDevices.size();
}

KinectManager::~KinectManager()
{
  mDevices.clear();
}

void KinectManager::startDepth()
{
  for (int i = 0; i < deviceCount(); ++i)
  {
    mDevices[i]->startDepth();
  }
}
void KinectManager::startVideo()
{
  for (int i = 0; i < deviceCount(); ++i)
  {
    mDevices[i]->startVideo();
  }
}
void KinectManager::stopDepth()
{
  for (int i = 0; i < deviceCount(); ++i)
  {
    mDevices[i]->stopDepth();
  }
}
void KinectManager::stopVideo()
{
  for (int i = 0; i < deviceCount(); ++i)
  {
    mDevices[i]->stopVideo();
  }
}

bool KinectManager::getDepth(int index, uint16_t **frame)
{
  return mDevices[index]->getDepthFrame(frame);
}

bool KinectManager::getVideo(int index, uint8_t **frame)
{
  return mDevices[index]->getVideoFrame(frame);
}


Kinect* KinectManager::getDevice(int index)
{
  return mDevices[index];
}
