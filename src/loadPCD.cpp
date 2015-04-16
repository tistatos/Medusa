
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>




using namespace cv;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadData();

int main(){
	//skapar ett "rum"
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//laddar filen
	cloud  = loadData();

	return 0;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr loadData(){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("data_test.pcd", *cloud) == -1)
	{
		std::cout << "failed to load file" << std::endl;
		return cloud;
	}
	std::cout << "file size " << std::endl;
	std::cout << "X:" << cloud->width << '\t' << "Y:" << cloud->height << std::endl;

	for (int i = 0; i < cloud->points.size (); ++i)
	{
	 	  std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
	}
	return cloud;
}