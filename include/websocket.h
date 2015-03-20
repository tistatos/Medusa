#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libwebsockets.h>


class Websocket {
  public:
    Websocket(int port, bool ssl = 0);
    int init();
    void run();
    void destroy();



  private: 
    struct libwebsocket_context *context;
    struct lws_context_creation_info info;
};