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
#include "kinect_depth.h"

/**
 * @brief [Get RGB Data]
 * @details [long description]
 * 
 * @param video [description]
 * @param timestamp [description]
 */

  void MyFreenectDevice::VideoCallback(void *video, uint32_t timestamp)
  {
    if(m_new_rgb_frame)
      return;
    mrgb = static_cast<uint8_t*>(video);
    copy(mrgb, mrgb+getVideoBufferSize(), m_buffer_video.begin());

    m_new_rgb_frame = true;
  }
/**
 * @brief [Get depth data]
 * @details [long description]
 * 
 * @param _depth [description]
 * @param timestamp [description]
 */
  void MyFreenectDevice::DepthCallback(void *_depth, uint32_t timestamp)
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
  /**
   * @brief [saves into a pcl::PointCloud]
   * @details [long description]
   * 
   * @param filename [description]
   */
  void MyFreenectDevice::savePointCloud(std::string filename)
  {
    //std::cout << cloud.points.size() << std::endl;
    pcl::io::savePCDFileASCII (filename, cloud);
  
  }


  