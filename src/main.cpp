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

  string option;
  std::cout << "Would you like to calibrate(y/n)?";
  std::cin >> option;
  if(option == "y")
  {
    medusa.init();
    std::cout << "Place origin and write \"ok\"";
    std::cin >> option;
    if(option == "ok")
      km.setOrigin();

    std::cout << "Origin Set";

  }
  else
  {
    //TODO: read data from an XML file
  }
  medusa.run();

  return 0;
}