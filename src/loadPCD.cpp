
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>
#include <pcl/pcl_base.h>
#include <pcl/io/ply_io.h>




using namespace cv;
using namespace std;


pcl::PointCloud<pcl::PointXYZ> loadData();

int main() {

	cout << "1" << endl;
    //skapar ett "rum"
    //pcl::PointCloud<pcl::PointXYZ>* cloud = 
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCDReader reader;
	reader.read<pcl::PointXYZ> ("data_test.pcd", cloud);
    cout << "2" << endl;

    loadData();

    //std::cout << "file size " << std::endl;
    //std::cout << "X:" << cloud->width << '\t' << "Y:" << cloud->height << std::endl;

 /*   for (int i = 0; i < cloud->points.size (); ++i)
    {
         //  std::cout << "    " << cloud->points[i].x
           //   << " "    << cloud->points[i].y
             // << " "    << cloud->points[i].z << std::endl;
    }*/

    //laddar filen

    return 0;
}

pcl::PointCloud<pcl::PointXYZ> loadData()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 1;
    cloud.height = 1;

    cout << "4" << endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data_test.pcd", cloud) == -1)
    {
    	cout << "5" << endl;
        std::cout << "failed to load file" << std::endl;
        //return NULL;
    }
    cout << "6" << endl;

    return cloud;
}




