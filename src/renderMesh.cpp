

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
#include <pcl/filters/voxel_grid.h>

using namespace cv;




  /**
   * @brief [Displays a pcl::PointCloud]
   * @details [long description]
   * 
   * @param d [description]
   */
  void renderMesh::show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
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

  /**
   * @brief [Displays a pcl PolygonMesh]
   * @details [long description]
   * 
   * @param mesh [description]
   */
  void renderMesh::showMesh (pcl::PolygonMesh mesh) 
  {
    pcl::visualization::PCLVisualizer viewer ("surface fitting");
    viewer.addPolygonMesh (mesh, "sample mesh");

    while (!viewer.wasStopped ())
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }

/**
 * @brief [Initiate class process]
 * @details [long description]
 * 
 * @param d [description]
 */
void renderMesh::run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
 /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2
      (new pcl::PointCloud<pcl::PointXYZ>(cloud));
*/
  //show(cloud); 

  cloud = reduceData(cloud);
  //show(cloud);
  std::cout << "cloud reduced" << endl; 
  //cloud = mirrorCloud(cloud);
  std::cout << "cloud mirrord" << endl;
  cloud = setDelims(cloud);
  //show(cloud);
  std::cout << "cloud Poisson" << endl;
  runPoisson(cloud);

}


  /**
   * @brief [Ŕeduce noise and nr of points]
   * @details [long description]
   * 
   * @param d [description]
   * @return [description]
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::reduceData (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    //octree compressos object
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ>octreeCompression (pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
    //stringstream that will hold the compressed cloud
    std::stringstream compressedData;
    //compress the cloud
    octreeCompression.encodePointCloud (cloud,compressedData);
    //Decompress the cloud
    octreeCompression.decodePointCloud (compressedData, decompressedCloud);

    return decompressedCloud; */

    //Voxel Grid
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //convert to pointCloud2
    pcl::toPCLPointCloud2(*cloud, *cloud2); //rätt håll?

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    sor.filter (*cloud_blob);

    //convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);

    return cloud_filtered;
  }



/**
 * @brief [Returns normals for a pcl::PointCloud]
 * @details [long description]
 * 
 * @param d [description]
 * @return [description]
 */

pcl::PointCloud<pcl::PointNormal>::Ptr renderMesh::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setRadiusSearch (5);
  //n.setRadiusSearch(5);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.compute (*normals);
 

  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudWithNormals);
  
  std::cout << "normaler klar" << std::endl;
  return cloudWithNormals;

}
/**
 * @brief [Builds a mesh from a PointCloud]
 * @details [long description]
 * 
 * @param d [description]
 */
void renderMesh::runPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  /*
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (200);
 // n.setRadiusSearch(0.03);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.compute (*normals);
 

 // std::cout << "normals " << normals->size() << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudAndNormals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudAndNormals);


  pcl::PolygonMesh mesh;

  //pcl::GridProjection<pcl::PointNormal> gp;
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;


  gp3.setSearchRadius(0.5);
  gp3.setMu(3);
  gp3.setMaximumNearestNeighbors(200);
  gp3.setMaximumSurfaceAngle(M_PI/4);
  gp3.setMinimumAngle(M_PI/18);
  gp3.setMaximumAngle(2*M_PI/3);
  gp3.setNormalConsistency(false);

  gp3.setInputCloud(cloudAndNormals);
  gp3.setSearchMethod(tree2);
  //gp.setResolution(1);

  gp3.reconstruct(mesh);

  showMesh(mesh);

  //pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  //pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  //tree2->setInputCloud(getNormals(cloud));

*/
 
  pcl::Poisson<pcl::PointNormal> poisson;

  //poisson.setSearchMethod(tree2);
  poisson.setInputCloud (getNormals (cloud));
  pcl::PolygonMesh mesh;
  poisson.reconstruct (mesh);
 // pcl::io::saveOBJFile ("bunny_poisson.obj", mesh); 
  showMesh (mesh);
  
}
/**
 * @brief [Removes points outside a set span]
 * @details [long description]
 * 
 * @param  [description]
 * @return [description]
 */

pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::setDelims(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond ( new pcl::ConditionAnd<pcl::PointXYZ>);
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -2.1)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.5)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.5)));
 
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(true);
  condrem.filter(*cloud);
  

  return cloud;
}
/**
 * @brief [Mirrors a cloud]
 * @details [long description]
 * 
 * @param d [description]
 * @return [description]
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::mirrorCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(2, 2) = -1;
  transform(2, 3) = 2.405; //z
  transform(1, 3) = -0.05;  //y
  //transform(0, 3) = 0.03; //x

  std::cout << "reducerat" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

 
  return transformed_cloud;
}

