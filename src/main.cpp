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
    return -1;
  }
  else
  {
    km.connectToDevices();
    std::cout << "Connected to " << km.getConnectedDeviceCount() << " devices." << std::endl;
  }

  Websocket ws(7681);
  ws.init();
  Medusa medusa(&km, &ws);

  string option;

  while(true)
  {
    std::cout << "Would you like to recalibrate(y/n)? ";
    std::cin >> option;
    if(option == "y")
    {
      medusa.init();
      break;
    }
    else
    {
      std::cout << "Trying to load data from files..." << std::endl;

      if(!km.loadCalibration())
      {
        std::cout << "Failed ot load calibration data" << std::endl;
      }
      else
      {
        std::cout << "Calibration data loaded" << std::endl;
        break;
      }
    }

  }


  std::cout << "Place origin and write \"ok\" ";
  std::cin >> option;
  if(option == "ok")
    km.setOrigin();

  std::cout << "Origin Set" << std::endl;

  std::cout << "Awaiting Connection" << std::endl;

  medusa.run();

  return 0;
}
