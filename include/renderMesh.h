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


class renderMesh
{
	public:

		static void run(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static void show (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

		static void showMesh(pcl::PolygonMesh mesh);

		static void runPoisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static  pcl::PointCloud<pcl::PointNormal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static  void runGreedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static  pcl::PointCloud<pcl::PointXYZ>::Ptr setDelims(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static  pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static  pcl::PointCloud<pcl::PointXYZ>::Ptr reduceData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static  pcl::PointCloud<pcl::PointXYZ>::Ptr mirrorCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		static pcl::PointCloud<pcl::PointXYZ>::Ptr smoothing (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};

