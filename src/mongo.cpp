
#include "mongo.h"

using namespace std;

void mongo::storeObject(string fileName)
{
	mongo::client::initialize();
	mongo::DBClientConnection c;
	c.connect("localhost");

	mongo::GridFS gfs = mongo::GridFS(c, "testet");
	gfs.storeFile(fileName);

	if(gfs.findFileByName(fileName) == fileName)
	{
		cout << "found it" << endl;
	}


//I think it calls the destructor for the connection when it leaves the function. /Carl
}

void mongo::getObject()
{
	
}