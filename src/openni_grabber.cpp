/**
 * @File openni_grabber.cpp
 *    quick test with OpenNI and kinect
 * @autor Erik Sandrén
 * @date 2015-03-05
 */

#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


/**
 * @brief Simple point cloud viewer connected to kinect
 */
class SimpleOpenNIProcessor
{

public:
     SimpleOpenNIProcessor () : viewer ("PCL OpenNI Viewer") {}

     /**
      * @brief Callback function for openNI
      * @details gets point cloud from kinect with color and shows it in the view.
      */
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     /**
      * @brief starts interface and hooks callback to data from interface
      *
      */
     void run ()
     {
      //Create interface
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       //create a function pointe to cloud_cb_
       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);
       //register callback function to interface
       interface->registerCallback (f);
       //Start capture on interface
       interface->start ();

       //running loop
       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }
       //stop interface
       interface->stop ();
     }
     //The point cloud viewer
     pcl::visualization::CloudViewer viewer;

/*public:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cld;
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static unsigned pictureNumber = 1;
    static double last = pcl::getTime ();
    if (++count == 30)
    {
      double now = pcl::getTime ();
      std::cout << "number of points: " << cloud->size() << std::endl;
      std::cout << "Width: " << cloud->width << std::endl;
      std::cout << "Height: " << cloud->height << std::endl;
      //pcl::io::savePCDFileASCII(pictureNumber + "test_pcd.pcd", *cloud);
      //pictureNumber++;
      count = 0;
      last = now;
      cld = cloud;
    }
  }

  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (!viewer.wasStopped ())
    {
      viewer.showCloud(cld);
    }
    // stop the grabber
    interface->stop ();
  }*/
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}