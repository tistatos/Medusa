/**
 * @File kinect_depth.cpp
 *    test with async kinect
 * @autor Erik Sandrén
 * @date 2015-03-06
 */

#ifndef __KINECT_H__
#define __KINECT_H__

#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <libfreenect/libfreenect.hpp>
#include <libfreenect/libfreenect.h>
#include <string.h>

#include <pthread.h>


class Mutex
{
public:
    Mutex()
    {
        pthread_mutex_init(&m_mutex, NULL);
    }

    void lock()
    {
        pthread_mutex_lock(&m_mutex);
    }

    void unlock()
    {
        pthread_mutex_unlock(&m_mutex);
    }

    class ScopedLock
    {
    public:
        ScopedLock(Mutex &mutex) : _mutex(mutex)
        {
            _mutex.lock();
        }

        ~ScopedLock()
        {
            _mutex.unlock();
        }

    private:
        Mutex &_mutex;
    };

private:
    pthread_mutex_t m_mutex;
};



class Kinect : public Freenect::FreenectDevice
{
public:
  Kinect(freenect_context* ctx, int index);
  ~Kinect();

  bool getVideoFrame(uint8_t **frame);
  bool getDepthFrame(uint16_t **frame);

  void VideoCallback(void *video, uint32_t timestamp);
  void DepthCallback(void *_depth, uint32_t timestamp);

  void savePointCloud(std::string filename);
  pcl::PointCloud<pcl::PointXYZ> getPointCloud();

private:
  bool mNewRgbFrame;
  bool mNewDepthFrame;
  pcl::PointCloud<pcl::PointXYZ> mCloud;
  uint16_t* mBufferDepth;
  uint8_t* mBufferVideo;
  Mutex mRgbMutex;
  Mutex mDepthMutex;
};

#endif