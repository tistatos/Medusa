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
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
/**
 * @brief Simple point cloud viewer connected to kinect
 */
class SimpleOpenNIProcessor
{

public:
     SimpleOpenNIProcessor () : viewer ("PCL OpenNI Viewer") {}
     Eigen::Affine3f transform_1;
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;
     /**
      * @brief Callback function for openNI
      * @details gets point cloud from kinect with color and shows it in the view.
      */
     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
      pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);
      pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr range_cond ( new pcl::ConditionAnd<pcl::PointXYZRGBA>);
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::LT, 2)));

      pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem (range_cond);
      condrem.setInputCloud(transformed_cloud);
      condrem.setKeepOrganized(true);
      condrem.filter(*transformed_cloud);

      if (!viewer.wasStopped())
        viewer.showCloud (transformed_cloud);

     }

     /**
      * @brief starts interface and hooks callback to data from interface
      *
      */
     void run ()
     {
      transform_1 = Eigen::Affine3f::Identity();
      float theta = M_PI;
      transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));


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