

#include "renderMesh.h"
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>

#include <pcl/compression/octree_pointcloud_compression.h>

using namespace cv;

class renderMesh
{
  public:

  void run()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(loadData()));

    cloud = mirrorCloud(cloud);
    cloud = reduceData(cloud);
    cloud = setDelims(cloud);
    runGp3(cloud);
  }

  //Read PCD file
  pcl::PointCloud<pcl::PointXYZ> loadData()
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    //Read file and test
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("model.pcd", cloud) == -1)
    {
      std::cout << "failed to load file" << std::endl;
      return cloud;
    }
    return cloud;
  }

  //Visualize Cloud data
  void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }

  //Visualize Mesh data
  void showMesh (pcl::PolygonMesh mesh) 
  {
    pcl::visualization::PCLVisualizer viewer ("surface fitting");
    viewer.addPolygonMesh (mesh, "sample mesh");

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }

  //Calculate Normals
  pcl::PointCloud<pcl::PointNormal>::Ptr getNormals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    tree->setInputCloud (cloud);
    
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (3);
    n.compute (*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloudWithNormals);
    
    std::cout << "Normals done" << std::endl;
    return cloudWithNormals;
  }

  //Triangulation of unordered point cloud
  void runGp3 (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (getNormals (cloud));

    //initialize
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangle;

    //set constraints
    gp3.setSearchRadius (0.05);
    gp3.setMu (5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle (M_PI); // 45 degrees
    gp3.setMinimumAngle (M_PI / 18); // 10 degrees
    gp3.setMaximumAngle (2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency (false);
    gp3.setInputCloud (getNormals(cloud));
    gp3.setSearchMethod (tree2);      

    std::cout << "reconstruct mesh" << std::endl;
    gp3.reconstruct (triangle);

    //saves to an obj file
    //pcl::io::saveOBJFile ("mesh.obj", triangle, 5);
    std::cout << "show mesh" << std::endl;

    showMesh (triangle);
  }

  //Filtering Data: Background and sides
  pcl::PointCloud<pcl::PointXYZ>::Ptr setDelims (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>);
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.7)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.3)));
   
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized (true);
    condrem.filter (*cloud);

    return cloud;
  }

  //Sample Down data
  pcl::PointCloud<pcl::PointXYZ>::Ptr reduceData (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    //octree compressos object
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ>octreeCompression (pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
    //stringstream that will hold the compressed cloud
    std::stringstream compressedData;
    //compress the cloud
    octreeCompression.encodePointCloud (cloud,compressedData);
    //Decompress the cloud
    octreeCompression.decodePointCloud (compressedData, decompressedCloud);

    return decompressedCloud; 
  }

  //Mirroring of the Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr mirrorCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (2, 2) = -1;
    std::cout << "Reduced" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    return transformed_cloud;
  }


  private:

};

