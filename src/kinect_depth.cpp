/**
 * @File kinect_depth.cpp
 *    test with async kinect
 * @autor Erik Sandrén
 * @date 2015-03-06
 */

#include <vector>
#include <iostream>
#include "libfreenect.hpp"

Freenect::Freenect freenect;

class MyFreenectDevice : public Freenect::FreenectDevice
{
public:
  MyFreenectDevice(freenect_context *ctx, int index) : Freenect::FreenectDevice(ctx,index),
    m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
    m_buffer_depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes / 2),
    m_new_rgb_frame(false), m_new_depth_frame(false)
  {
    setDepthFormat(FREENECT_DEPTH_REGISTERED);
  }

  void VideoCallback(void *video, uint32_t timestamp)
  {
    std::cout << "Video!" << std::endl;
  }

  void DepthCallback(void *video, uint32_t timestamp)
  {
  }
private:
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_buffer_depth;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
};

int main(int argc, char const *argv[])
{
  MyFreenectDevice* device;
  device = &freenect.createDevice<MyFreenectDevice>(0);
  return 0;
}