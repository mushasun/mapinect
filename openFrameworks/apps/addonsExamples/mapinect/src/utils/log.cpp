#include "log.h"

#include <assert.h>
#include <sstream>

#include "ofxMutex.h"
#include "Timer.h"

static std::string		logs[kLogFileCount];
static mapinect::Timer	logsTimer[kLogFileCount];
static ofxMutex			logsMutex;

void log(const LogFile& file, const std::string& str)
{
	assert(file < kLogFileCount);

	ofxScopedMutex osm(logsMutex);
	logsTimer[file].end();
	ostringstream oss;
	oss << logsTimer[file].getElapsedSeconds() << "\t" << str << endl;
	logs[file] += oss.str();
	logsTimer[file].start();
}

void printLogFileToFile(const LogFile& file, const std::string& filename, bool clear)
{
	assert(file < kLogFileCount);

	ofstream ofs;
	ofs.open(filename.c_str());
	ofxScopedMutex osm(logsMutex);
	ofs << logs[file] << "print" << endl;
	ofs.close();
	if (clear)
		logs[file] = "";
	logsTimer[file].start();
}

void printLogFile(const LogFile& file, bool clear)
{
	assert(file < kLogFileCount);

	ofxScopedMutex osm(logsMutex);
	cout << logs[file] << "print" << endl;
	if (clear)
		logs[file] = "";
	logsTimer[file].start();
}

static std::string	pcmThreadStatus = "";
static std::string	objectsThreadStatus = "";
static ofxMutex		pcmThreadStatusMutex;
static ofxMutex		objectsThreadStatusMutex;

void setPCMThreadStatus(const std::string& status)
{
	ofxScopedMutex osm(pcmThreadStatusMutex);
	pcmThreadStatus = status;
}

std::string getPCMThreadStatus()
{
	ofxScopedMutex osm(pcmThreadStatusMutex);
	string result(pcmThreadStatus);
	return result;
}

void setObjectsThreadStatus(const std::string& status)
{
	ofxScopedMutex osm(objectsThreadStatusMutex);
	objectsThreadStatus = status;
}

std::string getObjectsThreadStatus()
{
	ofxScopedMutex osm(objectsThreadStatusMutex);
	string result(objectsThreadStatus);
	return result;
}
