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

	logsMutex.lock();
	ostringstream oss;
	oss << logsTimer[file].stopResumeAndGetElapsedSeconds() << "\t" << str << endl;
	logs[file] += oss.str();
	logsMutex.unlock();
}

void printLogFileToFile(const LogFile& file, const std::string& filename, bool clear)
{
	assert(file < kLogFileCount);

	ofstream ofs;
	ofs.open(filename.c_str());
	logsMutex.lock();
	ofs << logs[file] << "print" << endl;
	ofs.close();
	if (clear)
		logs[file] = "";
	logsTimer[file].start();
	logsMutex.unlock();
}

void printLogFile(const LogFile& file, bool clear)
{
	assert(file < kLogFileCount);

	logsMutex.lock();
	cout << logs[file] << "print" << endl;
	if (clear)
		logs[file] = "";
	logsTimer[file].start();
	logsMutex.unlock();
}

static std::string	pcmThreadStatus = "";
static std::string	objectsThreadStatus = "";
static ofxMutex		pcmThreadStatusMutex;
static ofxMutex		objectsThreadStatusMutex;

void setPCMThreadStatus(const std::string& status)
{
	pcmThreadStatusMutex.lock();
	pcmThreadStatus = status;
	pcmThreadStatusMutex.unlock();
}

std::string getPCMThreadStatus()
{
	pcmThreadStatusMutex.lock();
	string result(pcmThreadStatus);
	pcmThreadStatusMutex.unlock();
	return result;
}

void setObjectsThreadStatus(const std::string& status)
{
	objectsThreadStatusMutex.lock();
	objectsThreadStatus = status;
	objectsThreadStatusMutex.unlock();
}

std::string getObjectsThreadStatus()
{
	objectsThreadStatusMutex.lock();
	string result(objectsThreadStatus);
	objectsThreadStatusMutex.unlock();
	return result;
}
