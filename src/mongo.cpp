
#include "mongo.h"



void mongo::storeFile(string fileName)
{
	mongo::client::initialize();
	mongo::DBClientConnection c;
	c.connect("localhost");

	mongo::GridFS gfs = mongo::GridFS(c, "testet");
	gfs.storeFile(fileName);


//I think it calls the destructor for the connection when it leaves the function. /Carl
}