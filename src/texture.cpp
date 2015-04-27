
#include "texture.h"

using namespace pcl;

// KOMMER INTE ATT BEHÖVA showCameras I SENARE SKEDE
/**
 * @brief [Visualize cameras.]
 * @details [Visualize cameras in window.]
 *
 * @param cams [The camera vectors representing the camera position.]
 * @param d [The mesh which we want to apply our texture to.]
 */
/** \brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses */
void Texture::showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

  // visualization object
  //pcl::visualization::PCLVisualizer visu ("cameras");

  // add a visual for each camera at the correct pose
  for(int i = 0 ; i < cams.size () ; ++i)
  {
    // read current camera
    pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
    double focal = cam.focal_length;
    double height = cam.height;
    double width = cam.width;

    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
    double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist*tan (atan (width / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (atan (height / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
    p1=pcl::transformPoint (p1, cam.pose);
    p2=pcl::transformPoint (p2, cam.pose);
    p3=pcl::transformPoint (p3, cam.pose);
    p4=pcl::transformPoint (p4, cam.pose);
    p5=pcl::transformPoint (p5, cam.pose);
    /*
    std::stringstream ss;
    ss << "Cam #" << i+1;
    visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

    ss.str ("");
    ss << "camera_" << i << "line1";
    visu.addLine (p1, p2,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line2";
    visu.addLine (p1, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line3";
    visu.addLine (p1, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line4";
    visu.addLine (p1, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line5";
    visu.addLine (p2, p5,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line6";
    visu.addLine (p5, p4,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line7";
    visu.addLine (p4, p3,ss.str ());
    ss.str ("");
    ss << "camera_" << i << "line8";
    visu.addLine (p3, p2,ss.str ());
  */
  }
  

  // add a coordinate system
  //visu.addCoordinateSystem (1.0);

  // add the mesh's cloud (colored on Z axis)
  //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
  //visu.addPointCloud (cloud, color_handler, "cloud");

  // reset camera
  //visu.resetCamera ();

  // wait for user input
  //visu.spin ();
}

/**
 * @brief [Get camera positions]
 * @details [Supposed to read from calibration file later on.]
 *
 * @param g [The camera]
 */
/** \brief Helper function that reads a camera file outputed by Kinfu */
void Texture::readCamPoseFile(pcl::TextureMapping<pcl::PointXYZ>::Camera &cam)
{

  // camera position
  cam.pose (0,3) = 0; //TX
  cam.pose (1,3) = 0; //TY
  cam.pose (2,3) = 0; //TZ

  // rotation coordinates
  cam.pose (0,0) = 1;
  cam.pose (0,1) = 0;
  cam.pose (0,2) = 0;

  cam.pose (1,0) = 0;
  cam.pose (1,1) = 1;
  cam.pose (1,2) = 0;

  cam.pose (2,0) = 0;
  cam.pose (2,1) = 0;
  cam.pose (2,2) = 1;

  cam.pose (3,0) = 0.0;
  cam.pose (3,1) = 0.0;
  cam.pose (3,2) = 0.0;
  cam.pose (3,3) = 1.0; //Scale

  // camera focal length and size
  cam.focal_length=525;
  cam.height=480;
  cam.width=640;
}

/**
 * @brief [The run function for texturing]
 * @details [Where everything happens!]
 */
void Texture::applyTexture(pcl::PolygonMesh &triangles, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) // Att skicka med: PolygonMesh, pcl::PointCloud<pcl::PointNormal>
{
  //PCL_INFO ("\nLoading mesh from file %s...\n", "file.obj");
  std::cout << "Loading mesh... " << std::endl;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //fromPCLPointCloud2 (triangles.cloud, *cloud);

  // Create the texturemesh object that will contian our UV-mapped mesh
  TextureMesh mesh;
  mesh.cloud = triangles.cloud;
  std::vector<pcl::Vertices> polygon_1;

  // push faces into the texturemesh object
  polygon_1.resize (triangles.polygons.size ());
  for(size_t i =0; i < triangles.polygons.size (); ++i)
  {
    polygon_1[i] = triangles.polygons[i];
  }
  mesh.tex_polygons.push_back(polygon_1);
  PCL_INFO ("\tInput mesh contains %d faces and %d vertices\n", mesh.tex_polygons[0].size (), cloud->points.size ());
  PCL_INFO ("...Done.\n");

  // Load textures and cameras poses and intrinsics
  PCL_INFO ("\nLoading textures and camera poses...\n");
  pcl::texture_mapping::CameraVector my_cams;

  int cpt_cam = 0;
  int numberOfCams = 1;
  for (int i = 0; i < numberOfCams; i++)
  {
    pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
    readCamPoseFile(cam);
    cam.texture_file = "0_bild.png";
    my_cams.push_back (cam);
    cpt_cam++ ;
  }

  PCL_INFO ("\tLoaded %d textures.\n", my_cams.size ());
  PCL_INFO ("...Done.\n");

  // Display cameras to user
  PCL_INFO ("\nDisplaying cameras. Press \'q\' to continue texture mapping\n");
  showCameras(my_cams, cloud);

  // Create materials for each texture (and one extra for occluded faces)
  mesh.tex_materials.resize (my_cams.size () + 1);
  for(int i = 0 ; i <= my_cams.size() ; ++i)
  {
    pcl::TexMaterial mesh_material;
    mesh_material.tex_Ka.r = 0.2f;
    mesh_material.tex_Ka.g = 0.2f;
    mesh_material.tex_Ka.b = 0.2f;

    mesh_material.tex_Kd.r = 0.8f;
    mesh_material.tex_Kd.g = 0.8f;
    mesh_material.tex_Kd.b = 0.8f;

    mesh_material.tex_Ks.r = 1.0f;
    mesh_material.tex_Ks.g = 1.0f;
    mesh_material.tex_Ks.b = 1.0f;

    mesh_material.tex_d = 1.0f;
    mesh_material.tex_Ns = 75.0f;
    mesh_material.tex_illum = 2;

    std::stringstream tex_name;
    tex_name << "material_" << i;
    tex_name >> mesh_material.tex_name;

    if(i < my_cams.size ())
      mesh_material.tex_file = my_cams[i].texture_file;
    else
      mesh_material.tex_file = "occluded.jpg";

    mesh.tex_materials[i] = mesh_material;
  }

  // Sort faces
  PCL_INFO ("\nSorting faces by cameras...\n");
  pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
  tm.textureMeshwithMultipleCameras(mesh, my_cams);


  PCL_INFO ("Sorting faces by cameras done.\n");
  for(int i = 0 ; i <= my_cams.size() ; ++i)
  {
    PCL_INFO ("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size (), mesh.tex_coordinates[i].size ());
  }

  // compute normals for the mesh
  PCL_INFO ("\nEstimating normals...\n");
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  PCL_INFO ("...Done.\n");

  pcl::toPCLPointCloud2 (*cloud_with_normals, mesh.cloud);

  PCL_INFO ("\nSaving mesh to file.obj\n");

  pcl::io::saveOBJFile("file.obj", mesh);

  /*
  pcl::visualization::PCLVisualizer viewer ("surface fitting");
    viewer.addTextureMesh (mesh, "sample mesh");

    while (!viewer.wasStopped ())
    {
      //viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  */
}