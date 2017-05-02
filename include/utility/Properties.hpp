#ifndef _PROPERTIES
#define _PROPERTIES

#include <iostream>
#include <map>
#include <string>
#include <string.h> //strncmp
#include <fstream>
#include <stdlib.h> // atof

using namespace std;

class Properties {
public:
	Properties(string filename);
	string getStringForKey(string key);
	double getDoubleForKey(string key);
	int getIntForKey(string key);
	bool getBoolForKey(string key);

private:
	typedef map<const string, string> configurationMapping;
	configurationMapping configs;
	double stringToDouble(string value);
	int stringToInt(string value);
	bool stringToBoolean(string value);
};

#endif //_PROPERTIES
