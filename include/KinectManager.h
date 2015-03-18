/**
 * @File KinectManager.h
 *    description here
 * @autor Erik Sandrén
 * @date  DATE
 */
#ifndef __KINECT_MANAGER_H__
#define __KINECT_MANAGER_H__

#include "libfreenect/libfreenect.h"
#include "libfreenect/libfreenect_sync.h"
#include "Kinect.h"

#include <stddef.h>
#include <vector>


class KinectManager
{
public:
  KinectManager();
  int getDeviceCount();
  void connectToDevice(int index=-1);

private:
  freenect_context* mCtx;
  std::vector<Kinect> mDevices;
};




#endif
