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
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include "libfreenect/libfreenect.hpp"
#include <string.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include "renderMesh.h"
#include "kinect_depth.h"

//nrKinects is howmany - 1 to fit indexing.
const int nrKinects = 0;
Freenect::Freenect freenect;
MyFreenectDevice* device;



FILE *open_dump(const char *filename)
{
  //open file
  FILE* fp = fopen(filename, "w");
  if (fp == NULL) {
    fprintf(stderr, "Error: Cannot open file [%s]\n", filename);
    exit(1);
  }
  printf("%s\n", filename);
  return fp;
}

void dump_rgb(FILE *fp, void *data, unsigned int width, unsigned int height)
{

  //*3 = channel
  fprintf(fp, "P6 %u %u 255\n", width, height);
  //write to file
  fwrite(data, width * height * 3, 1, fp);
}

pcl::PointCloud<pcl::PointXYZ> runAllKinects(int kinectIndex)
{
  device = &freenect.createDevice<MyFreenectDevice>(nrKinects);

  device->startDepth();
  device->startVideo();


  while(!device->m_new_rgb_frame && !device->m_new_depth_frame)
  {
    //run loop as long as we dont have depth data
    std::cout << "running..." << std::endl;
  }

  device->stopDepth();
  device->stopVideo();

  device->savePointCloud("first.pcd");

  return device->cloud;
}


int main(int argc, char const *argv[])
{

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > v;


  for(int i = 0; i<= nrKinects; i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(runAllKinects(i)));
    v.push_back(cloud);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAll (new pcl::PointCloud<pcl::PointXYZ>());
  //important to know index of all kinects to preform transforms
  // transforms goes here

  for(int i = 0; i<= nrKinects; i++)
  {
    *cloudAll = *cloudAll + *v[i];
  }

  renderMesh::run(cloudAll);


  return 0;
}

