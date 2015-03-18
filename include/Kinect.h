/**
 * @File Kinect.h
 *    description here
 * @autor Erik Sandrén
 * @date  DATE
 */
#ifndef __KINECT_H__
#define __KINECT_H__

#include "libfreenect/libfreenect.h"

class Kinect
{
public:
  Kinect() {};
  ~Kinect() {};
  freenect_device* getDevice();
  void setIndex(int index);

private:
  freenect_device* mDev;
  int mIndex;
};

#endif