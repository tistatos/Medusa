#ifndef __RENDERMESH_H__
#define __RENDERMESH_H__

#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>
#include <boost/thread/thread.hpp>

class mongo
{
public:

	static void storeObject(string fileName);
	static void getObeject();
	
};

#endif