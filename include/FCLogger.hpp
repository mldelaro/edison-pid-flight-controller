/*
 * FCLogger.h
 *
 *  Created on: Mar 11, 2017
 *      Author: mldof
 */

#ifndef INCLUDE_FCLOGGER_HPP_
#define INCLUDE_FCLOGGER_HPP_

#include <fstream>
#include <iostream>
#include <ctime>
#include <string>
#include <cstdio>
#include <iomanip>
#include <ctime>
#include <unistd.h> // for use with symlink
#include <stdio.h> //for use with remove file

class FCLogger {
private:
	char timeStamp[50];
	int csvColumnCount;
	std::string logDirectory;
	std::string logFilename;


public:
	FCLogger(std::string fileDirectory, std::string fileName, std::string fileExtension);
	virtual ~FCLogger();

	void startCsvLog(int columnCount, const char** columnNames);
	std::ofstream logStream;
};

#endif /* INCLUDE_FCLOGGER_HPP_ */
