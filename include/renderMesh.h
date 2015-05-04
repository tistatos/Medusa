#ifndef __RENDERMESH_H__
#define __RENDERMESH_H__

#include <iostream>
#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include "websocket.h"


class renderMesh
{
	public:

		static pcl::PointCloud<pcl::PointXYZ>::Ptr run(pcl::PolygonMesh &mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

		static void showMesh(pcl::PolygonMesh mesh);

		static void runPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

		static  pcl::PointCloud<pcl::PointNormal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

		static  void runGreedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

		static pcl::PointCloud<pcl::PointXYZ>::Ptr setDelims(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static pcl::PointCloud<pcl::PointXYZ>::Ptr reduceData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static pcl::PointCloud<pcl::PointXYZ>::Ptr mirrorCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static pcl::PointCloud<pcl::PointXYZ>::Ptr smoothing (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static void storeFile();

		static std::string currentDateTime();

		static int hash( const string &key, int tableSize);

		static std::string getHash();

};

#endif