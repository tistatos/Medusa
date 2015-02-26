#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>

using namespace cv;

int main(int argc, char const *argv[])
{
  will destroy
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 10;
  cloud.height   = 10;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;


  Mat image;
  image = imread("dick.png", CV_LOAD_IMAGE_COLOR);   // Read the file

  if(! image.data )                              // Check for invalid input
  {
      std::cout <<  "Could not open or find the image" << std::endl ;
      return -1;
  }


  freenect_context* fn_ctx;
  int ret = freenect_init(&fn_ctx, NULL);
  std::cout << ret;
  if (ret < 0)
    return ret;

  // Show debug messages and use camera only.
  freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);

  return 0;
}
