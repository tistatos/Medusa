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

	
   MyFreenectDevice(freenect_context* ctx, int index): Freenect::FreenectDevice(ctx,index),
    m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
    m_buffer_depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes / 2),
    m_new_rgb_frame(false), m_new_depth_frame(false){

    setDepthFormat(FREENECT_DEPTH_REGISTERED);
    setVideoFormat(FREENECT_VIDEO_RGB);

    cloud.width = 640;
    cloud.height = 480;
    cloud.points.resize(640*480);

    }

  void VideoCallback(void *video, uint32_t timestamp);

  void DepthCallback(void *_depth, uint32_t timestamp);

  void savePointCloud(std::string filename);
  
public:
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_buffer_depth;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  uint8_t* mrgb;
};
