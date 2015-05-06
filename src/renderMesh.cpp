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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = cloud;
    show(cloud);
    cloud = setDelims(cloud);
    //show(cloud);
    std::cout << "Delims set" << std::endl;
    cloud = reduceData(cloud);
    std::cout << "Data reduced" << std::endl;
    cloud = removeNoise(cloud);
    std::cout << "Noise removed" << endl;
   
    //using with poisson smoothes away points 
    //that are needed for reconstruction
    //cloud = smoothing(cloud);
   
    runPoisson(cloud, cloud2);
    //runGreedyProjectionTriangulation(cloud);
    std::cout << "GP3 done." << endl;
    //storeFile();
    std::cout << "Finished" << endl;
    return cloud;
  }

  
  /**
  * @brief Displays a pcl::PointCloud
  * @param pcl::PointXYZ::ConstPtr 
  */
  //Visualize Cloud data
  void renderMesh::show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }
 

  /**
  * @brief Displays a PolygoMesh
  * @param mesh pcl::PolygonMesh
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
  * @brief Remove noise from a pointcloud using stdandarddeviation, 
  * @param pcl::PointXYZ::Ptr 
  * @return pcl::PointXYZ::Ptr 
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::removeNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    //Removing outliers using a statisticalOutlierRemoval filter
    //Create the filtering object

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud);
    sor2.setMeanK (20);
    sor2.setStddevMulThresh (1.0);
    sor2.filter (*cloud_filtered);

    return cloud_filtered;
  }
  
  /**
   * @brief Reduce A pointCloud using a VoxelGrid filter
   * @param pcl::PointXYZ::Ptr 
   * @return pcl::PointXYZ::Ptr 
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
  * @brief Suface smoothing using moving least squares
  * @param pcl::PointXYZ::Ptr 
  * @return pcl::PointXYZ::Ptr 
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
   * @brief Returns normals for a pcl::PointCloud and points dem towards the origin
   * @param a pcl::PointXYZ::Ptr
   * @return pcl::Normal>::Ptr 
   */
  pcl::PointCloud<pcl::PointNormal>::Ptr renderMesh::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    n.setInputCloud (cloud);

    // Pass the original data (before downsampling) as the search surface
    n.setSearchSurface (cloud2);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given surface dataset.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    n.setSearchMethod (tree);

    n.setRadiusSearch (0.03);
    //n.setRadiusSearch (5);
    n.compute(*normals);
    std::cout << "alla sets klara"<<std::endl;

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud,*normals, *cloudWithNormals);
   
    for(int i = 0; i < cloudWithNormals->size();i++){
       pcl::flipNormalTowardsViewpoint (cloud->points[i], 
        0,
        0, 
        0, 
        cloudWithNormals->points[i].x,
        cloudWithNormals->points[i].y,
        cloudWithNormals->points[i].z


         );
    }
    
    std::cout << "normaler klar" << std::endl;

    std::cout << "normaler klar" << std::endl;
 
    return cloudWithNormals;
  }
  /**
  * @brief Build a surface using GPT, 
  * @param  descriptionpcl::PointCloud<pcl::PointXYZ>::Ptr 
  */

  void renderMesh::runGreedyProjectionTriangulation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)

  {
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    
    tree2->setInputCloud (getNormals(cloud,cloud2));

  //  pcl::PolygonMesh mesh;

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    gp3.setSearchRadius (0.5);
    gp3.setMu (3);
    gp3.setMaximumNearestNeighbors (200);
    gp3.setMaximumSurfaceAngle (M_PI / 4);
    gp3.setMinimumAngle (M_PI / 18);
    gp3.setMaximumAngle (2 * M_PI / 3);
    gp3.setNormalConsistency (false);

    gp3.setInputCloud (getNormals(cloud,cloud2));
    gp3.setSearchMethod (tree2);
    //gp.setResolution(1);

    //gp3.reconstruct(mesh);

    std::cout << "runGreedyProjectionTriangulation done!" << endl;
    //pcl::io::saveOBJFile("file.obj", mesh);

    //showMesh(mesh);
  }
  
  
  /**
   * @brief Builds a surface using poisson surface reconstruction
   * @param a pcl::PointXYZ::ptr
   */
  void renderMesh::runPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
  {
    pcl::Poisson<pcl::PointNormal> poisson;

    //poisson.setSearchMethod(tree2);
    poisson.setInputCloud (getNormals (cloud, cloud2));
    pcl::PolygonMesh mesh;

    poisson.reconstruct (mesh);
    poisson.setDepth(18);
    //pcl::io::saveOBJFile("file.obj", mesh);
    std::cout << "cloud Poisson" << endl;

  }
  

  /**
   * @brief Set max size of a cloud, max and min(x,y,z)
   * @param  pcl::PointXYZ::ptr
   * @return pcl::PointXYZ::ptr
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr renderMesh::setDelims(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond ( new pcl::ConditionAnd<pcl::PointXYZ>);

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, 1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -1)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, 0.5)));

    pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*cloud);

    return cloud;
  }
  

  /**
   * @brief Mirrors a cloud using a transformation matrix
   *
   * @param pcl::PointXYZ::ptr
   * @return pcl::PointXYZ::ptr
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
