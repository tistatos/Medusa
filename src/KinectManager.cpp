/**
 * @File KinectManager.cpp
 *    description here
 * @autor Erik Sandrén
 * @date  DATE
 */

#include "KinectManager.h"


KinectManager::KinectManager()
{
  int ret = freenect_init(&mCtx, NULL);
  freenect_select_subdevices(mCtx, FREENECT_DEVICE_CAMERA);
}

int KinectManager::getDeviceCount()
{
  return freenect_num_devices(mCtx);
}

void KinectManager::connectToDevice(int index)
{

  if(index == -1)
  {
    int deviceCount = getDeviceCount();
    for(int i = 0; i < deviceCount; ++i)
    {
      freenect_device* dev = mDevices[i].getDevice();
      int ret = freenect_open_device(mCtx, &dev, i);
      mDevices[i].setIndex(i);
    }
  }
}

