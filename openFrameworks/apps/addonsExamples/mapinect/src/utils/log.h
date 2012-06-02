#ifndef LOG_H__
#define LOG_H__

#include <string>

enum LogFile
{
	kLogFilePCMThread = 0,
	kLogFileObjectsThread,
	kLogFileCount
};

void log(const LogFile& file, const std::string& str);
void printLogFileToFile(const LogFile& file, const std::string& filename, bool clear = true);
void printLogFile(const LogFile& file, bool clear = true);

void setPCMThreadStatus(const std::string& status);
std::string getPCMThreadStatus();

void setObjectsThreadStatus(const std::string& status);
std::string getObjectsThreadStatus();

#endif	// LOG_H__