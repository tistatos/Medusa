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


class MyFreenectDevice : public Freenect::FreenectDevice
{
public:
  MyFreenectDevice(freenect_context *ctx, int index) : Freenect::FreenectDevice(ctx,index),
    m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
    m_buffer_depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes / 2),
    m_new_rgb_frame(false), m_new_depth_frame(false)
  {
    setDepthFormat(FREENECT_DEPTH_REGISTERED);
    cloud.width = 640;
    cloud.height = 480;
    cloud.points.resize(480*640);
  }

  void VideoCallback(void *video, uint32_t timestamp)
  {
    uint8_t* rgb = static_cast<uint8_t*>(video);
    copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
  }

  void DepthCallback(void *_depth, uint32_t timestamp)
  {
     uint16_t* depth = static_cast<uint16_t*>(_depth);
    copy(depth, depth+getDepthBufferSize()/2, m_buffer_depth.begin());

    float f = 59500.f;
    for (int i = 0; i < 480*640; ++i)
    {
      cloud.points[i].x = (i%640 - (640-1)/2.f) * depth[i] / f;  // X = (x - cx) * d / fx
      cloud.points[i].y = (i/640 - (480-1)/2.f) * depth[i] / f;  // Y = (y - cy) * d / fy
      cloud.points[i].z = depth[i]/1000.f;; // Z = d
    }
   m_new_depth_frame = true;
  }

  void savePointCloud()
  {
    std::cout << cloud.points.size() << std::endl;
    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  }
public:
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_buffer_depth;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
  pcl::PointCloud<pcl::PointXYZ> cloud;

};

Freenect::Freenect freenect;
MyFreenectDevice* device;


int main(int argc, char const *argv[])
{
  //get kinect count
  std::cout << "device count: " << freenect.deviceCount() << std::endl;

  //try to connect to first kinect
  device = &freenect.createDevice<MyFreenectDevice>(0);

  //start depthcallback
  device->startDepth();
  int i;

  while(!device->m_new_depth_frame)
  {
    //run loop as long as we dont have depth data
    std::cout << "running..." << std::endl;
  }
  //stop depth callback
  device->stopDepth();
  //save data to file
  device->savePointCloud();
  return 0;
}