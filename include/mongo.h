#ifndef __RENDERMESH_H__
#define __RENDERMESH_H__

#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>
#include <boost/thread/thread.hpp>
#include "MD5.h"

class mongo
{
public:

	static void storeObject(string fileName);
	
	static void getObject();
	
	static std::string getHash();

	static std::string currentDateTime();
};

#endif