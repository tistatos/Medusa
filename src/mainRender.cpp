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


Freenect::Freenect freenect;
MyFreenectDevice* device;
MyFreenectDevice* device2;
MyFreenectDevice* deviceSecond;


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




int main(int argc, char const *argv[])
{

    //get kinect count
    std::cout << "device count: " << freenect.deviceCount() << std::endl;
    std::cout << 1 << std::endl;
    //try to connect to first kinect
    device = &freenect.createDevice<MyFreenectDevice>(0);
    // deviceSecond = &freenect.createDevice<MyFreenectDevice>(1);

    //start depthcallback
    device->startDepth();
    device->startVideo();
    // deviceSecond->startDepth();
    // deviceSecond->startVideo();

    while(!device->m_new_rgb_frame && !device->m_new_depth_frame)
    {
      //run loop as long as we dont have depth data
      std::cout << "running..." << std::endl;
    }
    //stop depth callback
    // deviceSecond->stopDepth();
    // deviceSecond->stopVideo();
    device->stopDepth();
    device->stopVideo();


    //save data to file
    device->savePointCloud("first.pcd");
    // deviceSecond->savePointCloud("second.pcd");
    //FILE *fp1;
    // FILE *fp2;
    //fp1 = open_dump("bild1.ppm");
    //resolution 640x480
    //dump_rgb(fp1, device->mrgb, 640, 480);
    //close file
    //fclose(fp1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>(device->cloud));

     std::cout << "device count: " << freenect.deviceCount() << std::endl;
    std::cout << 0 << std::endl;
    //try to connect to first kinect
    device2 = &freenect.createDevice<MyFreenectDevice>(1);
    // deviceSecond = &freenect.createDevice<MyFreenectDevice>(1);

    //start depthcallback
    device2->startDepth();
    device2->startVideo();
    // deviceSecond->startDepth();
    // deviceSecond->startVideo();

    while(!device2->m_new_rgb_frame && !device2->m_new_depth_frame)
    {
      //run loop as long as we dont have depth data
      std::cout << "running..." << std::endl;
    }
    //stop depth callback
    // deviceSecond->stopDepth();
    // deviceSecond->stopVideo();
    device2->stopDepth();
    device2->stopVideo();


    //save data to file
    device2->savePointCloud("first.pcd");
    // deviceSecond->savePointCloud("second.pcd");
    //FILE *fp1;
    // FILE *fp2;
    //fp1 = open_dump("bild1.ppm");
    //resolution 640x480
    //dump_rgb(fp1, device->mrgb, 640, 480);
    //close file
    //fclose(fp1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>(device2->cloud));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>());
  renderMesh* r;

    cloud1 = r->mirrorCloud(cloud1);

  *cloud3 = *cloud2 + *cloud1;

  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(device->cloud));
  std::cout<<"lolo" << endl << endl;
  r->run(cloud3);

  // fp2 = open_dump("bild2.ppm");
  //resolution 640x480
  // dump_rgb(fp2, deviceSecond->mrgb, 640, 480);
  //close file
  // fclose(fp2);

  return 0;
}