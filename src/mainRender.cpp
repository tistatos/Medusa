#include "renderMesh.h"
#include "kinect_depth.h"

Freenect::Freenect freenect;
MyFreenectDevice* device;
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
  std::cout << atoi(argv[0]) << std::endl;
  //try to connect to first kinect
  device = &freenect.createDevice<MyFreenectDevice>(atoi(argv[0]));
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
  FILE *fp1;
  // FILE *fp2;
  fp1 = open_dump("bild1.ppm");
  //resolution 640x480
  dump_rgb(fp1, device->mrgb, 640, 480);
  //close file
  fclose(fp1);
  renderMesh* r;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(device->cloud));
  std::cout<<"lolo" << endl << endl;
  r->run(cloud2);

  // fp2 = open_dump("bild2.ppm");
  //resolution 640x480
  // dump_rgb(fp2, deviceSecond->mrgb, 640, 480);
  //close file
  // fclose(fp2);

  return 0;
}