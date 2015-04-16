/**
 * @file websocket.cpp
 *    websocket for medusa
 * @author Erik Sandrén
 * @date 2015-04-13
 */
#include "websocket.h"

//Apparently this is needed as a standard for the websocketlibrary.
//Needs to be the first protocol
static int callback_increment(struct libwebsocket_context *context,
                              struct libwebsocket *wsi,
                              enum libwebsocket_callback_reasons reason, void *user,
                              void *in, size_t len)

{
  return 0;
}

static int callback_http (struct libwebsocket_context * context,
              struct libwebsocket * wsi,
              enum libwebsocket_callback_reasons reason,
              void * user, void * in, size_t len)
{
  Websocket* ws = (Websocket*)libwebsocket_context_user(context);
  //Differents methods called for different events, such as send message
  //Connect to websocket and such
  //This code was "stolen" from an example found at libwebsockets. It takes the message
  //recieved and reverses the order of letters (then sends it as a responese)
  switch (reason) {
    case LWS_CALLBACK_ESTABLISHED: // Someone is connecting
      printf("connection established\n");
      ws->setInstance(wsi);
      break;
    case LWS_CALLBACK_RECEIVE: { // the funny part
      // log what we recieved and what we're going to send as a response.
      // that disco syntax `%.*s` is used to print just a part of our buffer
      // http://stackoverflow.com/questions/5189071/print-part-of-char-array
      printf("received data: %s", (char *) in);

      ws->setData((char*)in, len);
      break;
    }
    default:
      break;
  }
  return 0;
}

//Infostuff to be printed and set. Right now we are using port 7681.
//SSL is not activated.
static struct libwebsocket_protocols protocols[] = {
/* first protocol must always be HTTP handler */
{
  "http-only",    //protocol name
  callback_http,    // callback
  0             //per_session_data_size
},
{
  "increment-protocol",
  callback_increment,
  0
},
{ NULL, NULL, 0, 0 } // terminator
};



Websocket::Websocket(int port, bool ssl)
{
  mPort = port;
}

bool Websocket::init()
{
  mNewData = false;
  memset(&mInfo, 0, sizeof mInfo);

  mInfo.port = mPort;
  mInfo.protocols = protocols;
  mInfo.extensions = libwebsocket_get_internal_extensions();
  mInfo.ssl_cert_filepath = NULL;
  mInfo.ssl_private_key_filepath = NULL;
  mInfo.user = this;
  mInfo.gid = -1;
  mInfo.uid = -1;
  mInfo.options = 0;

  //Create the websocket context
  mContext = libwebsocket_create_context(&mInfo);

  //If context is null something is wrong, then the program should exit.
  if(mContext == NULL) {
    lwsl_err("Websocket failed to init");
    return false;
  }

  return true;
}

void Websocket::destroy()
{
  //When program is done destroy the websocket connection
  libwebsocket_context_destroy(mContext);

  //Notice that it exited correctly
  lwsl_notice("Websocket exited correctly");
}

int Websocket::RecieveData()
{
  //Run the webserver, ping with 50ms delay.
  return libwebsocket_service(mContext, 50);
}

bool Websocket::newData()
{
  return mNewData;
}


void Websocket::setData(char* buf, int len)
{
  mSocketData = buf;
  mNewData = true;
}

void Websocket::sendData(std::string data)
{
  unsigned char *buf = (unsigned char*) malloc(LWS_SEND_BUFFER_PRE_PADDING +
                                                data.size() + LWS_SEND_BUFFER_PRE_PADDING);
  for (int i = 0; i < data.size(); ++i)
  {
    buf[LWS_SEND_BUFFER_PRE_PADDING+i] = data[i];
  }
  libwebsocket_write(mWebsocketInstance, &buf[LWS_SEND_BUFFER_PRE_PADDING], data.size(), LWS_WRITE_TEXT);
  free(buf);
}

void Websocket::setInstance(struct libwebsocket* wsi)
{
  mWebsocketInstance = wsi;
}

void Websocket::setMedusa(Medusa* m)
{
  mMedusa = m;
}

std::string Websocket::getData()
{
  mNewData = false;
  return mSocketData;
}

void Websocket::startCountDown(int seconds)
{
  sendData("STARTCD 5");
}
