#ifndef __WEBSOCKET_H__
#define __WEBSOCKET_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <libwebsockets.h>

#include "Medusa.h"

class Medusa;

typedef void(Medusa::*MedusaFn)(int a);
class Websocket {
  public:
    Websocket(int port, bool ssl = 0);
    bool init();
    int RecieveData();
    void destroy();
    void setData(char* buf, int len);
    void setInstance(struct libwebsocket* wsi);
    void setMedusa(Medusa* m);
    bool newData();
    std::string getData();
    void startCountDown(int seconds);
  private:
    void sendData(std::string data);

    int mPort; /// listening port
    bool mNewData; /// status of socket, if new data has been recieved
    struct libwebsocket *mWebsocketInstance; /// websocket instance
    std::string mSocketData; /// newest sdata in socket
    struct libwebsocket_context *mContext; /// the context of the socket
    struct lws_context_creation_info mInfo; /// context creation info
    Medusa* mMedusa; /// medusa instance
};

#endif