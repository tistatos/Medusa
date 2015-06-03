#ifndef __MONGO_H__
#define __MONGO_H__

#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>
#include <boost/thread/thread.hpp>
#include "MD5.h"

class Mongo
{
public:
	static void storeObject(std::string modelID);

	static void getObject();

	static std::string getHash();

	static std::string currentDateTime();
};

#endif
