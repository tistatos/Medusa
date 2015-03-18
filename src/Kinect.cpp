/**
 * @File Kinect.cpp
 *    description here
 * @autor Erik Sandrén
 * @date  DATE
 */

 #include "Kinect.h"

 freenect_device* Kinect::getDevice()
 {
  return mDev;
 }

 void Kinect::setIndex(int index)
 {
  mIndex = index;
 }
