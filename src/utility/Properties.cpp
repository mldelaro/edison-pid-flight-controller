#include "../../include/utility/Properties.hpp"

using namespace std;

Properties::Properties(string filename) {
	ifstream propertyStream (filename);
	string propertyLine;
	if(propertyStream) {
		cout << "Reading from properties file " << filename << "..." << endl;
		while(std::getline(propertyStream, propertyLine)) {
			int delimPos = propertyLine.find('=');
			if(delimPos != string::npos) {
				configs[propertyLine.substr(0, delimPos)] = propertyLine.substr(delimPos + 1);
			}
		}
		for(configurationMapping::iterator iter = configs.begin(); iter != configs.end(); iter++) {
			cout << iter->first << " = " << iter->second << endl;
		}
	} else {
		std::cout << "Failed to open " << filename << std::endl;
	}
}

string Properties::getStringForKey(string key) {
	try {
		return configs.at(key);
	} catch(const std::out_of_range& ex) {
		std::cout << "Failed to get value for key " << key << std::endl;
		return "-1";
	} catch(int ex) {
		std::cout << "Failed to get value for key " << key << std::endl;
		return "-1";
	}
}

double Properties::getDoubleForKey(string key) {
	return std::atof(getStringForKey(key).c_str());
}

int Properties::getIntForKey(string key) {
	return std::atoi(getStringForKey(key).c_str());
}

bool Properties::getBoolForKey(string key) {
	if(strncmp(getStringForKey(key).c_str(), "true", 5) == 0) {
		return true;
	} else {
		return false;
	}
}
