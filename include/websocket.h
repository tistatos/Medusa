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
    void test();
    int RecieveData();
    void destroy();
    void sendData(std::string data);
    void setInstance(struct libwebsocket* wsi);
    void setMedusa(Medusa* m);
    void setData(char* buf, int len);
    bool newData();
    std::string getData();
    void startCountDown(int seconds);
  private:
    int mPort;
    bool mNewData;
    struct libwebsocket *mWebsocketInstance;
    std::string mSocketData;
    struct libwebsocket_context *mContext;
    struct lws_context_creation_info mInfo;
    Medusa* mMedusa;
};

#endif