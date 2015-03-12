
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





using namespace cv;


pcl::PointCloud<pcl::PointXYZ> loadData();

void TestPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void TestMC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void TestGp3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void showMesh(pcl::PolygonMesh mesh);

void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);


/**
 * @brief [main]
 * @details [long description]
 * @return [description]
 */

int main()

{
  //skapar ett "rum"

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud 
        (new pcl::PointCloud<pcl::PointXYZ>(loadData()));
show(cloud); 

  
//GreedyProjectionTriangulation      
TestGp3(cloud);

//Poisson
TestPoisson(cloud);

//Marching cubes
TestMC(cloud);
  
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

  if(pcl::io::loadPCDFile<pcl::PointXYZ> ("model.pcd", cloud) == -1)
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

//Estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);


  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud,*normals, *cloudWithNormals);


  //pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloudWithNormals);

  //initialize

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangle;

  gp3.setSearchRadius(0.025);

  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  gp3.setInputCloud (cloudWithNormals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangle);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  //saves to an obj file
  //pcl::io::saveOBJFile ("mesh.obj", triangle, 5);

  float ISOLevel = 0.5;
  float leafSize = 0.01f;
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
  
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
 
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (5);

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