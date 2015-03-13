
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


pcl::PointCloud<pcl::PointXYZ> loadData();
pcl::PointCloud<pcl::PointXYZ> loadData2();

void testGP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void TestPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void TestMC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void TestGp3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void showMesh(pcl::PolygonMesh mesh);

void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr reduceData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond ( new pcl::ConditionAnd<pcl::PointXYZ>);
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.1)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 0.7)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.3)));
 
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(true);
  condrem.filter(*cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  //octree compressos object
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ>octreeCompression(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
  //stringstream that will hold the compressed cloud
  std::stringstream compressedData;
  //compress the cloud
  octreeCompression.encodePointCloud(cloud,compressedData);
  //Decompress the cloud
  octreeCompression.decodePointCloud(compressedData, decompressedCloud);

  return decompressedCloud;
}

/**
 * @brief [main]
 * @details [long description]
 * @return [description]
 */

int main()
{
  //skapar ett "rum"

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFront 
      (new pcl::PointCloud<pcl::PointXYZ>(loadData()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBack 
      (new pcl::PointCloud<pcl::PointXYZ>(loadData2()));
  std::cout << cloudFront->size() << std::endl;
  cloudFront = reduceData(cloudFront);
  cloudBack = reduceData(cloudBack);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(2,2) = -1;
  
  transform(0,3) = 0.15;
  transform(1,3) = 0.1;
  transform(2,3) = 3;
  std::cout << "reducerat" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloudback (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloudBack, *transformed_cloudback, transform);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll (new pcl::PointCloud<pcl::PointXYZ>);
  *transformed_cloudback += *cloudFront;
  std::cout << transformed_cloudback->size() << std::endl;
  //show(transformed_cloudback); 

  //show(cloudFront); 

  //show(transformed_cloudback);
  
  //GreedyProjectionTriangulation     
  TestGp3(transformed_cloudback);

  //Poisson
  //TestPoisson(transformed_cloudback);

  //TestMC(cloud);
  //OFT
  //testGP(cloud);
  
  return 0;
}
/**
 * @brief [load Data]
 * @details [long description]
 * @return [description]
 */


pcl::PointCloud<pcl::PointXYZ> loadData()
{

 
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if(pcl::io::loadPCDFile<pcl::PointXYZ> ("carlfront.pcd", cloud) == -1)
  {
    std::cout << "failed to load file" << std::endl;
    return cloud;
  }

  return cloud;
  
}

pcl::PointCloud<pcl::PointXYZ> loadData2()
{

 
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if(pcl::io::loadPCDFile<pcl::PointXYZ> ("carlback.pcd", cloud) == -1)
  {
    std::cout << "failed to load file" << std::endl;
    return cloud;
  }

  return cloud;
  
}
/**
 * @brief [view pcl::Pointcloud]
 * @details [long description]
 * 
 * @param d [description]
 */

void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
 
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
 * @brief [View polygon mesh (surface)]
 * @details [long description]
 * 
 * @param mesh [description]
 */

void showMesh(pcl::PolygonMesh mesh){
    pcl::visualization::PCLVisualizer viewer ("surface fitting");
    viewer.addPolygonMesh(mesh, "sample mesh");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}
/**
 * @brief [Surface reconstruction using Greedy Projection Triangulation]
 * @details [long description]
 * 
 * @param d [description]
 */
void TestGp3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  
  std::cout << "normaler" << std::endl;
//Estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (3);
  n.compute (*normals);

  std::cout << "normaler klar" << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudWithNormals);


  //pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudWithNormals);

  //initialize

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangle;
  std::cout << "gp3 saker" << std::endl;
  gp3.setSearchRadius(0.05);

  gp3.setMu (5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  gp3.setInputCloud (cloudWithNormals);
  gp3.setSearchMethod (tree2);                                                     
  std::cout << "rekonstruera" << std::endl;
  gp3.reconstruct (triangle);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  //saves to an obj file
  //pcl::io::saveOBJFile ("mesh.obj", triangle, 5);
  std::cout << "show mesh" << std::endl;

  showMesh(triangle);
}

/**
 * @brief [Surface reconstruction using poisson]
 * @details [long description]
 * 
 * @param  [description]
 */
void TestPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  
  std::cout << cloud->size() << std::endl; 


  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  //n.setRadiusSearch(5);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.compute (*normals);
 

 // std::cout << "normals " << normals->size() << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudAndNormals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudAndNormals);



 
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setSearchMethod(tree2);
  poisson.setInputCloud (cloudAndNormals);
  pcl::PolygonMesh mesh;
  poisson.reconstruct (mesh);
 // pcl::io::saveOBJFile ("bunny_poisson.obj", mesh); 
  showMesh(mesh);
}

/**
 * @brief [Surface reconstruction using Marching cubes hoppe]
 * @details [long description]
 * 
 * @param  [description]
 */
void TestMC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (5);
 // n.setRadiusSearch(0.03);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.compute (*normals);
 

 // std::cout << "normals " << normals->size() << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudAndNormals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudAndNormals);


  pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
  pcl::PolygonMesh mesh;
  mc.setIsoLevel (0);
  mc.setGridResolution (100, 100, 100);
  mc.setPercentageExtendGrid (0.3f);
  mc.setInputCloud(cloudAndNormals);
  //std::vector<pcl::Vertices> vertices;
  mc.reconstruct(mesh);
  
  showMesh(mesh);
 

}

void testGP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (15);
 // n.setRadiusSearch(0.03);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.compute (*normals);
 

 // std::cout << "normals " << normals->size() << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudAndNormals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudAndNormals);


  pcl::PolygonMesh mesh;

  pcl::GridProjection<pcl::PointNormal> gp;

  gp.setInputCloud(cloudAndNormals);
  gp.setSearchMethod(tree2);
  gp.setResolution(0.001);
  gp.reconstruct(mesh);

  showMesh(mesh);

}