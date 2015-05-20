/**
 * @file KinectManager.h
 * @author Erik Sandrén
 * @date  2015-05-12
 * @brief  kinect manager
 */
#ifndef __KINECT_MANAGER_H__
#define __KINECT_MANAGER_H__

#define DEBUG true

#include <libfreenect/libfreenect.hpp>
#include <stddef.h>
#include <vector>
#include <string.h>

#include "Kinect.h"
#include "texture.h"

/**
 * Manager for multiple connected kinects
 */
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
  bool getVideo(int index, VIDEO_IMAGE &image);

  void calibratePosition();
  void setOrigin();
  bool getCalibrationStatus() { return mDevicesCalibrated; }

  bool getVideoStatus();
  bool getDepthStatus();
  Kinect* getDevice(int index);

private:
  bool mInitialized; /// Initialized status of manager
  std::vector<Kinect*> mDevices; ///list of all connected kinects
  bool mDevicesCalibrated; /// are the cameras calibrated?
};

#endif
