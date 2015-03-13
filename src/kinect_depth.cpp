/**
 * @File kinect_depth.cpp
 *    test with async kinect
 * @autor Erik Sandrén
 * @date 2015-03-06
 */

#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "libfreenect/libfreenect.hpp"
#include <string.h>

class MyFreenectDevice : public Freenect::FreenectDevice
{
public:
  MyFreenectDevice(freenect_context *ctx, int index) : Freenect::FreenectDevice(ctx,index),
    m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
    m_buffer_depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes / 2),
    m_new_rgb_frame(false), m_new_depth_frame(false)
  {
    setDepthFormat(FREENECT_DEPTH_REGISTERED);
    setVideoFormat(FREENECT_VIDEO_RGB);

    cloud.width = 640;
    cloud.height = 480;
    cloud.points.resize(640*480);
  }

  void VideoCallback(void *video, uint32_t timestamp)
  {
    if(m_new_rgb_frame)
      return;
    mrgb = static_cast<uint8_t*>(video);
    copy(mrgb, mrgb+getVideoBufferSize(), m_buffer_video.begin());

    m_new_rgb_frame = true;
  }

  void DepthCallback(void *_depth, uint32_t timestamp)
  {
    if(m_new_depth_frame)
      return;
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    copy(depth, depth+getDepthBufferSize()/2, m_buffer_depth.begin());

    float f = 595000.f;
    for (int i = 0; i < 640*480; ++i)
    {
      cloud.points[i].x = (i%640 - (640-1)/2.f) * depth[i] / f;  // X = (x - cx) * d / fx
      cloud.points[i].y = (i/640 - (480-1)/2.f) * depth[i] / f;  // Y = (y - cy) * d / fy
      cloud.points[i].z = depth[i]/1000.f; // Z = d
    }
    m_new_depth_frame = true;
  }

  void savePointCloud(std::string filename)
  {
    std::cout << cloud.points.size() << std::endl;
    pcl::io::savePCDFileASCII (filename, cloud);
  }

public:
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_buffer_depth;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  uint8_t* mrgb;
};

Freenect::Freenect freenect;
MyFreenectDevice* device;
MyFreenectDevice* deviceSecond;



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

  //get kinect count
  std::cout << "device count: " << freenect.deviceCount() << std::endl;
  std::cout << atoi(argv[1]) << std::endl;
  //try to connect to first kinect
  device = &freenect.createDevice<MyFreenectDevice>(atoi(argv[1]));
  // deviceSecond = &freenect.createDevice<MyFreenectDevice>(1);

  //start depthcallback
  device->startDepth();
  device->startVideo();
  // deviceSecond->startDepth();
  // deviceSecond->startVideo();

  while(!device->m_new_rgb_frame && !device->m_new_depth_frame)
  {
    //run loop as long as we dont have depth data
    std::cout << "running..." << std::endl;
  }
  //stop depth callback
  // deviceSecond->stopDepth();
  // deviceSecond->stopVideo();
  device->stopDepth();
  device->stopVideo();


  //save data to file
  device->savePointCloud("first.pcd");
  // deviceSecond->savePointCloud("second.pcd");
  FILE *fp1;
  // FILE *fp2;
  fp1 = open_dump("bild1.ppm");
  //resolution 640x480
  dump_rgb(fp1, device->mrgb, 640, 480);
  //close file
  fclose(fp1);


  // fp2 = open_dump("bild2.ppm");
  //resolution 640x480
  // dump_rgb(fp2, deviceSecond->mrgb, 640, 480);
  //close file
  // fclose(fp2);

  return 0;
}