/**
 * @file medusaLogic.cpp
 *    The logic of medusa
 * @author Erik Sandrén
 * @date 2015-05-13
 */

#include "Medusa.h"
#include "renderMesh.h"
#include "texture.h"
#include "mongo.h"
#include "png++/png.hpp"

/**
 * @brief open file for writing
 *
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
 *
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

Medusa::~Medusa()
{
}

/**s
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

        if(mManager->getDepthStatus() && mManager->getVideoStatus())
        {
          //Save images from cameras
          for (int i = 0; i < mManager->getConnectedDeviceCount(); ++i)
          {
            uint8_t* frame;
            FILE *fp;
            mManager->getVideo(i, &frame);

            char filename[128];
            sprintf(filename, "%i_bild.png", i);

            std::cout << "Saving image to: " << filename << std::endl;

            fp = open_dump(filename);
           // dump_rgb(fp, frame, 640, 480);

            std::cout << "skapar image" << std::endl;
            png::image< png::rgb_pixel > image(640,480);
            //png::image<png::index_pixel> image;
            std::cout << "skapar bild"<< std::endl;
            for(int h = 0; h < 640*480*3; h+=3)
            {
                int y = h/(640*3);
                int x = (h/3)%640;
                image[y][x] = png::rgb_pixel(frame[h], frame[h+1], frame[h+2]);
            }

            std::cout<<"skriver till bild"<< std::endl;
            image.write(filename);


            fclose(fp);

            delete[] frame;
            *cloud2 = *cloud2 + mManager->getDevice(i)->getPointCloud();
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>(mManager->getDevice(i)->getPointCloud()));
            //pcl::io::savePCDFile("file.obj", mManager->getDevice(i)->getPointCloud());
            //renderMesh::show(cloud2);
          }
          for(int i = 0; i < mManager->getConnectedDeviceCount(); i++)
          {
            mManager->getDevice(i)->getPointCloud();
          }

          //Saves cloud2 to temp/file.obj in renderMesh::run
          pcl::PolygonMesh mesh;
          cloud2 = renderMesh::run(mesh, cloud2);
          Texture::applyTexture(mesh, cloud2);
          //mongo::storeFile("file.obj");
        }
        else
        {

        }
      }
    }
  }
}