
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>
#include <pcl/opencv_grabber.h>



using namespace cv;

bool loadData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

int main(){
	//skapar ett "rum"
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//laddar filen
	if(!loadData(cloud)){
		std::cout << "failed to load file" << std::endl;
  		return 0;
	}
	



	return 0;
}


bool loadData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("data_test.pcd", *cloud) == -1)
	{
		//PCLERROR("no file");
		return false;
	}
	std::cout << "file size " << std::endl;
	std::cout << "X:" << cloud->width << '\t' << "Y:" << cloud->height << std::endl;

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
	 	  std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
	}
	return true;
}