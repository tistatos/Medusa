/**
 * @file kinect_depth.cpp
 *    test with async kinect
 * @author Erik Sandrén
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
#include <opencv2/core/core.hpp>

#include <pthread.h>

/**
 * @brief Mutex class for locking and unlocking
 */

class Mutex
{
 public:
  /**
  * @brief default constructor
  */
  Mutex()
  {
    pthread_mutex_init(&m_mutex, NULL);
  }
  /**
   * @brief lock the mutex lock
   */
  void lock()
  {
    pthread_mutex_lock(&m_mutex);
  }
  /**
   * @brief unlock the mutex lock
   */
  void unlock()
  {
    pthread_mutex_unlock(&m_mutex);
  }

  /**
   * @brief Class to create mutex lock for the current scope
   */
  class ScopedLock
  {
  public:
    /**
     * @brief default constructor
     */
    ScopedLock(Mutex &mutex) : _mutex(mutex)
    {
      _mutex.lock();
    }
    /**
     * @brief default destructor
     */
    ~ScopedLock()
    {
      _mutex.unlock();
    }

  private:
     Mutex &_mutex; ///variable for mutex status
  };
private:
  pthread_mutex_t m_mutex; ///pthread mutex
};


/**
 * @brief Kinect class handles ineraction and callback to one kinect
 */
class Kinect : public Freenect::FreenectDevice
{
public:
  Kinect(freenect_context* ctx, int index);
  ~Kinect();

  bool getVideoFrame(uint8_t **frame);
  bool getDepthFrame(uint16_t **frame);

  bool getVideoStatus();
  bool getDepthStatus();

  void savePointCloud(std::string filename);
  pcl::PointCloud<pcl::PointXYZ> getPointCloud();

  void setPosition(cv::Point3f newPosition);
  cv::Point3f getPosition();

protected:
  void VideoCallback(void *video, uint32_t timestamp);
  void DepthCallback(void *_depth, uint32_t timestamp);
private:
  bool mNewRgbFrame; ///true if new frame is present since last runt of getVideoFrame
  bool mNewDepthFrame; ///true if new frame is present since last runt of getDepthFrame
  pcl::PointCloud<pcl::PointXYZ> mCloud; ///internal cloud represntation from depth data
  uint16_t* mBufferDepth; ///depth buffer
  uint8_t* mBufferVideo; ///video buffer
  Mutex mRgbMutex; //mutex lock for video data (write/read)
  Mutex mDepthMutex; //mutex lock for depth data (write/read)

  cv::Point3f mPosition; //the kinect's position from the center
};

#endif