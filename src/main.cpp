/**
 * @file Kinect.h
 *    description here
 * @author Erik Sandrén
 * @date  DATE
 */

#include "KinectManager.h"
#include "renderMesh.h"
#include <iostream>

FILE *open_dump(const char *filename)
{
  //open file
  FILE* fp = fopen(filename, "w");
  if (fp == NULL) {
    fprintf(stderr, "Error: Cannot open file [%s]\n", filename);
    exit(1);
  }
  printf("%s\n", filename);
  return fp;
}

void dump_rgb(FILE *fp, void *data, unsigned int width, unsigned int height)
{

  //*3 = channel
  fprintf(fp, "P6 %u %u 255\n", width, height);
  //write to file
  fwrite(data, width * height * 3, 1, fp);
}

int main(int argc, char const *argv[])
{
  KinectManager km;
  int devCount = km.getDeviceCount();
  std::cout << "Number of discovered kinects " << devCount << std::endl;

  if(devCount <= 0)
  {
    std::cout << "No kinects found!" << std::endl;
    return 0;
  }

  km.connectToDevices();
  std::cout << "Connected to " << km.getConnectedDeviceCount() << " devices." << std::endl;


  km.startVideo();
  km.startDepth();


  //Save images from cameras
  for (int i = 0; i < km.getConnectedDeviceCount(); ++i)
  {
    uint8_t* frame;
    uint16_t* dFrame;
    FILE *fp;
    while(!km.getVideo(i, &frame))
    {
      std::cout << "waiting for image" << std::endl;

    }
    char filename[128];
    sprintf(filename, "%i_bild.ppm", i);
    std::cout << "Saving image to: " << filename << std::endl;

    fp = open_dump(filename);
    dump_rgb(fp, frame, 640, 480);
    fclose(fp);

    while(!km.getDepth(i, &dFrame))
    {
      std::cout << "waiting for depth frame" << std::endl;
    }

    sprintf(filename, "%i_depth.pcd", i);
    km.getDevice(i)->savePointCloud(filename);

    delete[] frame;
    delete[] dFrame;
  }

  km.stopDepth();
  km.stopVideo();

  renderMesh r;
  for (int i = 0; i < km.getConnectedDeviceCount(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(km.getDevice(i)->getPointCloud()));
    r.show(cloud2);
  }

  return 0;
}