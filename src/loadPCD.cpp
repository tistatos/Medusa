
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



using namespace cv;
using namespace std;




int main(){
	//skapar ett "rum"
	//(int*)std::malloc(4*sizeo->f(int));
	//pcl::PointCloud<pcl::PointXYZRGBA> a = new pcl::PointCloud<pcl::PointXYZRGBA>();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>());
	if(pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_1.pcd", *cloud) == -1)
	{
		std::cout << "failed to load file" << std::endl;
		//return NULL;
	}
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr a ;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	viewer->setBackgroundColor (0, 0, 0);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>rgb(cloud);


	viewer->addPointCloud<pcl::PointXYZRGB> (cloud,rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	
	while (!viewer->wasStopped ())
	{
	 	viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	
	//laddar filen*/
	return 0;
}



