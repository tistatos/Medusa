/**
 * @file Kinect.h
 *    description here
 * @author Erik Sandrén
 * @date  DATE
 */

#include "KinectManager.h"
#include "Medusa.h"
#include <iostream>

int main(int argc, char const *argv[])
{
  KinectManager km;

  int devCount = km.getDeviceCount();
  std::cout << "Number of discovered kinects " << devCount << std::endl;

  if(devCount <= 0)
  {
    std::cout << "No kinects found!" << std::endl;
  }
  else
  {
    km.connectToDevices();
    std::cout << "Connected to " << km.getConnectedDeviceCount() << " devices." << std::endl;
  }


  Websocket ws(7681);
  ws.init();
  Medusa medusa(&km, &ws);
  //medusa.init(); //REMOVE COMMENT TO RUN WITH CALIBRATION
  medusa.run();

  return 0;
}