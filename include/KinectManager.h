/**
 * @File KinectManager.h
 *    description here
 * @autor Erik Sandrén
 * @date  DATE
 */
#ifndef __KINECT_MANAGER_H__
#define __KINECT_MANAGER_H__

#define DEBUG true

#include <libfreenect/libfreenect.hpp>

#include "Kinect.h"
#include "debug_helpers.h"

#include <stddef.h>
#include <vector>
#include <string.h>

class KinectManager : Freenect::Freenect
{
public:
  KinectManager();
  ~KinectManager();
  int getDeviceCount();
  void connectToDevices();
  int getConnectedDeviceCount();

  void startDepth();
  void startVideo();

  void stopDepth();
  void stopVideo();

  bool getDepth(int index, uint16_t **frame);
  bool getVideo(int index, uint8_t **frame);

  Kinect* getDevice(int index);

private:
  bool mInitialized;
  std::vector<Kinect*> mDevices;
};

#endif
