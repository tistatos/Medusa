
#include "mongo.h"

using namespace std;

string Mongo::storeObject()
{
	mongo::client::initialize();
	mongo::DBClientConnection c;
	c.connect("localhost");
    std::string modelID = getHash();
	std::system("zip " + modelID + ".zip file.* *_bild.png");

	mongo::GridFS gfs = mongo::GridFS(c, "testet");
	gfs.storeFile(modelID+".zip");

	if(gfs.findFileByName(modelID) == modelID)
	{
		cout << "found it" << endl;
	}
	return modelID;

}


void Mongo::getObject()
{
	
}

/**
* @brief Returns the current date as a char-string
*/
std::string Mongo::currentDateTime()
{
time_t     now = time(0);
struct tm  tstruct;
char       buf[80];
tstruct = *localtime(&now);
strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
return buf;
}


/**
* @brief getHash returns a md5-hash based on current time and the keyword "banan" as a std::string
* @return std::string
*/
std::string Mongo::getHash()
{

//Uses time and a keyword to create a modell id.
std::string timeHash = string(currentDateTime())+"banan";
std:: string hash = md5(timeHash);
return hash;
}
