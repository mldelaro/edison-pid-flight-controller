/*
 * FCLogger.cpp
 *
 *  Created on: Mar 11, 2017
 *      Author: mldof
 */

#include "../include/FCLogger.hpp"


FCLogger::FCLogger(std::string fileDirectory, std::string filename, std::string fileExtension) {
	csvColumnCount = 0;
	logFilename = filename;
	logDirectory = fileDirectory;

	std::time_t t = std::time(NULL);
	struct tm * timeOfInit = std::localtime(&t);
	std::strftime(timeStamp, 50, "%m-%d-%H-%M-%S", timeOfInit);

	std::string fullLogFilePath;
	fullLogFilePath.append(fileDirectory);
	fullLogFilePath.append(filename);
	fullLogFilePath.append("-");
	fullLogFilePath.append(timeStamp);
	fullLogFilePath.append(fileExtension);

	std::string symlinkToMostRecent;
	symlinkToMostRecent.append(fileDirectory);
	symlinkToMostRecent.append(filename);
	symlinkToMostRecent.append("-latest");

	std::cout << "Created log: " << fullLogFilePath << std::endl;

	logStream.open(fullLogFilePath);
	if(logStream.is_open()) {

		if(remove(symlinkToMostRecent.c_str()) == 0) {
			std::cout << "Updating: " << symlinkToMostRecent << std::endl;
		}

		symlink(fullLogFilePath.c_str(), symlinkToMostRecent.c_str());
		std::cout << "Created link to: " << symlinkToMostRecent << std::endl;

	} else {
		std::cout << "WARNING: Could not create CSV log file: " << fullLogFilePath << std::endl;
	}
}


void FCLogger::startCsvLog(int columnCount, const char** columnNames) {
	csvColumnCount = columnCount;

	for(int i = 0; i < columnCount; i++) {
		logStream << columnNames[i];
		if(i < (columnCount-1)) {
			logStream << ",";
		} else {
			logStream <<std::endl;
		}
	}
}

FCLogger::~FCLogger() {
	logStream.close();
	logStream.~basic_ofstream();
}
