//! @file websocket.cpp
//! Websocket file for medusa. Opens up a websocketserver for the C++ application
//!
//! @author Carl Englund
//! @version 1.0
//! @date 2015-03-19

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libwebsockets.h>


//Apparently this is needed as a standard for the websocketlibrary.
//Needs to be the first protocol
static int callback_http(struct libwebsocket_context *context,
                         struct libwebsocket *wsi,
                         enum libwebsocket_callback_reasons reason, void *user,
                         void *in, size_t len)
{
        //recieved and reverses the order of letters (then sends it as a responese)
    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED: // Someone is connecting
            printf("connection established\n");
            break;
        case LWS_CALLBACK_RECEIVE: { // the funny part
            // create a buffer to hold our response
            // it has to have some pre and post padding. You don't need to care
            // what comes there, libwebsockets will do everything for you. For more info see
            // http://git.warmcat.com/cgi-bin/cgit/libwebsockets/tree/lib/libwebsockets.h#n597
            unsigned char *buf = (unsigned char*) malloc(LWS_SEND_BUFFER_PRE_PADDING + len +
                                                         LWS_SEND_BUFFER_POST_PADDING);
           
            int i;
           
            // pointer to `void *in` holds the incomming request
            // we're just going to put it in reverse order and put it in `buf` with
            // correct offset. `len` holds length of the request.
            for (i=0; i < len; i++) {
                buf[LWS_SEND_BUFFER_PRE_PADDING + (len - 1) - i ] = ((char *) in)[i];
            }
           
            // log what we recieved and what we're going to send as a response.
            // that disco syntax `%.*s` is used to print just a part of our buffer
            // http://stackoverflow.com/questions/5189071/print-part-of-char-array
            printf("received data: %s, replying: %.*s\n", (char *) in, (int) len,
                 buf + LWS_SEND_BUFFER_PRE_PADDING);
           
            // send response
            // just notice that we have to tell where exactly our response starts. That's
            // why there's `buf[LWS_SEND_BUFFER_PRE_PADDING]` and how long it is.
            // we know that our response has the same length as request because
            // it's the same message in reverse order.
            libwebsocket_write(wsi, &buf[LWS_SEND_BUFFER_PRE_PADDING], len, LWS_WRITE_TEXT);
           
            // release memory back into the wild
            free(buf);
            break;
        }
        default:
            break;
    }
   
   
    return 0;
}

//Websocket protocols, can do own ones if wanted..
static struct libwebsocket_protocols protocols[] = {
	/* first protocol must always be HTTP handler */
	{
		"http-only",		//protocol name 
		callback_http,		// callback
		0			        //per_session_data_size 
	},
	{ NULL, NULL, 0, 0 } // terminator
};


/**
 * @brief Main function
 * @details Loads and starts a websocketserver. Runs until stopped by user.
 *
 * @return -1 if context is null
 */
int main(int argc, char **argv) {

	//Init variables needed
	int n = 0;
	struct libwebsocket_context *context;
	struct lws_context_creation_info info;


	memset(&info, 0, sizeof info);
	
	//Infostuff to be printed and set. Right now we are using port 7681.
	//SSL is not activated.
	info.port = 7681;
	info.protocols = protocols;
	info.extensions = libwebsocket_get_internal_extensions();
	info.ssl_cert_filepath = NULL;
	info.ssl_private_key_filepath = NULL;

	info.gid = -1;
	info.uid = -1;
	info.options = 0;

	//Create the websocket context
	context = libwebsocket_create_context(&info);

	//If context is null something is wrong, then the program should exit.
	if(context == NULL) {
		lwsl_err("Websocket failed to init");
		return -1;
	}


	//Run the webserver, ping with 50ms delay.
	while(true) {
		n = libwebsocket_service(context, 50);
	};

	//When program is done destroy the websocket connection
	libwebsocket_context_destroy(context);

	//Notice that it exited correctly
	lwsl_notice("Websocket exited correctly");

	return 0;


}