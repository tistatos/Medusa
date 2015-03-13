
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>

#include <pcl/pcl_base.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

#include <stdlib.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/compression/octree_pointcloud_compression.h>


using namespace cv;
using namespace std;


int main(int argc, char** argv) { 
    
    //skapar ett "rum"
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


    //Ta tillbaka sen
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud) == -1)
    {
        std::cout << "failed to load file" << std::endl;
        //return NULL;
    }

    //*****************REMOVE NOISE*********************************

    //Removing outliers using a statisticalOutlierRemoval filter
    //Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud Before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    //Outliers
    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);


    //********************** SAMPLE DOWN ****************************


    pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    //octree compressos object
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ>octreeCompression(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);

    //stringstream that will hold the compressed cloud
    std::stringstream compressedData;

    //compress the cloud
    octreeCompression.encodePointCloud(cloud_filtered,compressedData);
    //Decompress the cloud
    octreeCompression.decodePointCloud(compressedData, decompressedCloud);


    //*********************** VISUALIZATION  ****************************

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("Cloud"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("Cloud Filtered"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Cloud Decompressed"));

    viewer1->addPointCloud<pcl::PointXYZ>(cloud, "cloud"); //decompressedCloud
    viewer2->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered");
    viewer3->addPointCloud<pcl::PointXYZ>(decompressedCloud, "decompressed_cloud");

    while(!viewer1->wasStopped() && !viewer2->wasStopped())
    {
        viewer1->spinOnce(100);
        viewer2->spinOnce(100);
        viewer3->spinOnce(100);

        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}













