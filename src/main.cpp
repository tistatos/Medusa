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


  medusa.run();

  // km.startVideo();
  // km.startDepth();

  // while(!km.getDepthStatus() && !km.getVideoStatus())
  // {
  // }

  // km.stopDepth();
  // km.stopVideo();

  // //Save images from cameras
  // for (int i = 0; i < km.getConnectedDeviceCount(); ++i)
  // {
  //   uint8_t* frame;
  //   uint16_t* dFrame;
  //   FILE *fp;
  //   if(!km.getVideo(i, &frame))
  //   {
  //     std::cout << "Failed to grab video" << std::endl;
  //     return 1;
  //   }
  //   char filename[128];
  //   sprintf(filename, "%i_bild.ppm", i);
  //   std::cout << "Saving image to: " << filename << std::endl;

  //   fp = open_dump(filename);
  //   dump_rgb(fp, frame, 640, 480);
  //   fclose(fp);

  //   while(!km.getDepth(i, &dFrame))
  //   {
  //     std::cout << "Failed to grab video" << std::endl;
  //     return 1;
  //   }

  //   sprintf(filename, "%i_depth.pcd", i);
  //   km.getDevice(i)->savePointCloud(filename);

  //   delete[] frame;
  //   delete[] dFrame;
  // }

  // renderMesh r;
  // for (int i = 0; i < km.getConnectedDeviceCount(); ++i)
  // {
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(km.getDevice(i)->getPointCloud()));
  //   r.show(cloud2);
  // }

  return 0;
}