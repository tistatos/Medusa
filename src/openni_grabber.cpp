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
#include <pcl/filters/voxel_grid.h>

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
       
       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;

       pcl::VoxelGrid<pcl::PointXYZRGBA> filter;

       filter.setInputCloud(cloud);

       filter.filter(*filteredCloud);

       if (!viewer.wasStopped())
         viewer.showCloud (filteredCloud);

     }

     /**
      * @brief starts interface and hooks callback to data from interface
      *
      */
     void run ()
     {
      //Create interface
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       //create a function pointe to cloud_cb_1
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


};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}