/**
 * @file medusaLogic.h
 *   The logic of medusa
 * @author Erik Sandrén
 * @date 2015-05-13
 */

#ifndef __MEDUSALOGIC_H__
#define __MEDUSALOGIC_H__


#include "KinectManager.h"
#include "websocket.h"
#include "renderMesh.h"
#include "texture.h"


class Websocket;

class Medusa
{
public:
  Medusa(KinectManager* manager, Websocket* socket);
  ~Medusa();
  void init();
  void run();
  void stop();
  bool running();
  void saveImages();

private:
  KinectManager* mManager; /// kinect manager
  Websocket* mSocket; /// the socket
  bool mRunning; /// running status of medusa
};
#endif