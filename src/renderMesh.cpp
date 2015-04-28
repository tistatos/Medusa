#include "renderMesh.h"
#include <mongo/client/gridfs.h>


using namespace cv;

  /**
   * @brief Initiate renderMesh
   *
   * @param d description
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::run(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    std::cout << "Starting" << endl;

    cloud = setDelims(cloud);
    std::cout << "Delims set" << std::endl;
    cloud = removeNoise(cloud);
    std::cout << "Noise removed" << endl;
    cloud = reduceData(cloud);
    std::cout << "Data reduced" << std::endl;
    //cloud = smoothing(cloud);

    //runPoisson(cloud);
    runGreedyProjectionTriangulation(mesh, cloud);

    std::cout << "GP3 done." << endl;

    //storeFile("file.obj");
    std::cout << "Finished" << endl;

    return cloud;
  }

  /**
   * @brief Displays a pcl::PointCloud
   *
   * @param d description
   */
  //Visualize Cloud data
  void renderMesh::show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }
 /**
  * @brief Displays a PolygoMesh
  * @details long description
  *
  * @param mesh description
  */
  //Visualize Mesh data
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
   * @brief Remove Noise
   * @details long description0
   *
   * @param d description
   * @return description
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::removeNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    //Removing outliers using a statisticalOutlierRemoval filter
    //Create the filtering object

    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_filtered);*/

    return cloud;
  }

  /**
   * @brief Reduce A pointCloud
   * @details long description0
   *
   * @param d description
   * @return description
   */
  //Downsampling pointCloud using VoxelGrid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::reduceData (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //convert PointCloud to pointCloud2
    pcl::toPCLPointCloud2(*cloud, *cloud2);

    // Create the VoxelGrid filtering object
    //Leaf size is set to 0.5 cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    sor.filter (*cloud_blob);

    //convert to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);

    std::cout << "cloud reduced" << endl;

    return cloud_filtered;
  }

    /**
   * @brief Suface smoothing
   * @details long description0
   *
   * @param d description
   * @return description
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::smoothing (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr smoothedNormalCloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>);


    //Smoothing object
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
    filter.setInputCloud(cloud);
    filter.setSearchRadius(0.03);
    filter.setPolynomialFit(true);
    filter.setComputeNormals(true);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    filter.setSearchMethod(kdtree);

    filter.process(*smoothedNormalCloud);

    //convert pointNormal to pointCloud
    copyPointCloud(*smoothedNormalCloud, *cloud_smoothed);

    return cloud_smoothed;
  }


  /**
   * @brief Returns normals for a pcl::PointCloud
   * @details long description
   *
   * @param d description
   * @return description
   */
  pcl::PointCloud<pcl::PointNormal>::Ptr renderMesh::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setRadiusSearch (5);
    //n.setRadiusSearch (5);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    n.compute (*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud,*normals, *cloudWithNormals);

    std::cout << "normaler klar" << std::endl;

    return cloudWithNormals;
  }
  /**
   * @brief Build a surface using GPT
   * @details long description
   *
   * @param  description
   */
  void renderMesh::runGreedyProjectionTriangulation (pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (200);
    //n.setRadiusSearch(0.03);

    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    n.compute (*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud,*normals, *cloudAndNormals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloudAndNormals);

    //pcl::PolygonMesh mesh;

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    gp3.setSearchRadius (0.5);
    gp3.setMu (3);
    gp3.setMaximumNearestNeighbors (200);
    gp3.setMaximumSurfaceAngle (M_PI / 4);
    gp3.setMinimumAngle (M_PI / 18);
    gp3.setMaximumAngle (2 * M_PI / 3);
    gp3.setNormalConsistency (false);

    gp3.setInputCloud (cloudAndNormals);
    gp3.setSearchMethod (tree2);
    //gp.setResolution(1);

    gp3.reconstruct(mesh);

    std::cout << "runGreedyProjectionTriangulation done!" << endl;
    //pcl::io::saveOBJFile("file.obj", mesh);

    //showMesh(mesh);
  }
  /**
   * @brief Builds a surface using poisson
   * @details long description
   *
   * @param d description
   */
  void renderMesh::runPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::Poisson<pcl::PointNormal> poisson;

    //poisson.setSearchMethod(tree2);
    poisson.setInputCloud (getNormals (cloud));
    pcl::PolygonMesh mesh;
    poisson.reconstruct (mesh);

    std::cout << "cloud Poisson" << endl;

  }
  /**
   * @brief Reduce a cloud with set parameters
   * @details long description
   *
   * @param  description
   * @return description
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
   * @brief Mirrors a cloud
   * @details long description
   *
   * @param d description
   * @return description
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::mirrorCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(2, 2) = -1;
    transform(2, 3) = 2.405;  //z
    transform(1, 3) = -0.05;  //y
    //transform(0, 3) = 0.03; //x

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    std::cout << "cloud mirrored" << endl;

    return transformed_cloud;
  }
  /**
   * @brief Inserts file into Mongo
   * @details long description
   *
   * @param d description
   * @return description
   */

