/**
 * @file medusaLogic.cpp
 *    The logic of medusa
 * @author Erik Sandrén
 * @date 2015-05-13
 */

#include "Medusa.h"

/**
 * @brief open file for writing
 * @deprecated save as png instead
 * @param filename name of file
 * @return pointer to file
 */
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

/**
 * @brief Save rgb image to file
 * @deprecated save as png instead
 * @param fp file
 * @param data data
 * @param int width of image
 * @param int height
 */
void dump_rgb(FILE *fp, void *data, unsigned int width, unsigned int height)
{

  //*3 = channel
  fprintf(fp, "P6 %u %u 255\n", width, height);
  //write to file
  fwrite(data, width * height * 3, 1, fp);
}

/**
 * @brief Default constructor
 *
 * @param manager kinectmanager to use, should have connected to kinects already
 * @param socket websocket to use
 */
Medusa::Medusa(KinectManager* manager, Websocket* socket)
{
  mManager = manager;
  mSocket = socket;
  mRunning = false;

  socket->setMedusa(this);
}


void Medusa::init()
{
  mManager->calibratePosition();
}

Medusa::~Medusa()
{
}

/**
 * @brief Save kinect rgb images to hardrive
 * @todo  perhaps a nicer filename?
 */
void Medusa::saveImages(string modelID)
{
  for (int i = 0; i < mManager->getConnectedDeviceCount(); ++i)
  {

    VIDEO_IMAGE image(640,480);
    mManager->getVideo(i, image);
    Texture::updateTextureFiles(mManager->getDevice(i));
    string filename = mManager->getDevice(i)->getFilename();
    cv::Mat cvImage = Calibration::fromPNGtoMat(image);
    cv::Mat newImage;
    cv::resize(cvImage, newImage, cv::Size(1024,1024),0,0, cv::INTER_CUBIC);

    cv::imwrite("./scans/" + filename, newImage);
  }
}

/**
 * @brief running loop of medusa
 */
void Medusa::run()
{
  mRunning = true;
  while(mRunning)
  {
    mSocket->RecieveData();

    if(mSocket->newData())
    {
      std::string newData = mSocket->getData();
      if(newData == "KINECTRGB")
      {
        mManager->startDepth();
        mManager->startVideo();
        //countdown
        mSocket->startCountDown(5);
        sleep(6);
        std::cout << "Five seconds" << std::endl;
        mManager->stopDepth();
        mManager->stopVideo();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
        std::cout << "Scanning done" << std::endl;

        string modelID = Mongo::getHash();
        mManager->setFilename(modelID);

        if(mManager->getDepthStatus() && mManager->getVideoStatus())
        {
          saveImages(modelID);
          for(int i = 0; i < mManager->getConnectedDeviceCount(); i++)
          {
            *cloud2 = *cloud2 + mManager->getDevice(i)->getPointCloud();
          }
          std::cout << "creating mesh" << std::endl;
          pcl::PolygonMesh mesh;
          cloud2 = renderMesh::run(mesh, cloud2);

          Texture::applyTexture(mesh, cloud2, modelID);

          Mongo::storeObject(modelID);
          std::cout << "Mesh construction Complete" << std::endl;
          mSocket->returnHashID(modelID);
        }
        else
        {
          std::cout << "No images :/" << std::endl;
        }
      }
      else if(newData == "EXITMEDUSA")
      {
        std::cout << "Exiting..." << std::endl;
        mRunning = false;
        return;
      }
      else
      {
        std::cout << newData << std::endl;

      }
    }
  }
}
