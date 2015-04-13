/**
 * @file renderMesh.h
 *    class to handle point cloud data and meshes
 * @author Erik Sandrén
 * @date 2015-05-12
 */
#ifndef __RENDERMESH_H__
#define __RENDERMESH_H__

#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/compression/octree_pointcloud_compression.h>



class renderMesh
{

public:

	void run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ> loadData(std::string filename);

	void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

	void showMesh(pcl::PolygonMesh mesh);

	pcl::PointCloud<pcl::PointNormal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	void runGp3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr setDelims(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr reduceData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr mirrorCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};

#endif
