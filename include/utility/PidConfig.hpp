#ifndef _PID_CONFIG
#define _PID_CONFIG

#include "./Properties.hpp"
#include "../FCConstants.hpp"

class PidConfig {
public:
	PidConfig();
	PidConfig(Properties* propertyFile);
	double* getProportionalGain();
	double* getIntegralGain();
	double* getDerivativeGain();
	double* getMaxPidOutput();
	int getMaxReceiverOutput();
	int getCorrectionFrequencyHz();
	bool isPidControllerLogEnabled();
	bool isPidSensoryLogEnabled();
	bool isGyroLogEnabled();

private:
	Properties* propertyFile;
	double PGain[3];
	double IGain[3];
	double DGain[3];
	double PIDMaxOutput[3];
	int maxReceiverOutput;
	int correctionFrequencyHz;
	bool pidControllerLogEnabled;
	bool pidSensoryGyroLogEnabled;
	bool gyroLogEnabled;

	void _getDoubleValue(double* val, std::string key) {
		double valueFromProperty;
		valueFromProperty = propertyFile->getDoubleForKey(key);
		if(valueFromProperty < 0) {
			*val = valueFromProperty;
		}
	}

	void _getIntValue(int* val, std::string key) {
		double valueFromProperty;
		valueFromProperty = propertyFile->getIntForKey(key);
		if(valueFromProperty < 0) {
			*val = valueFromProperty;
		}
	}

	/*
	std::string propertyKeys[] = {
		"p-gain-roll",
		"p-gain-pitch",
		"p-gain-yaw",
		"i-gain-roll",
		"i-gain-pitch",
		"i-gain-yaw",
		"d-gain-roll",
		"d-gain-pitch",
		"d-gain-yaw",
		"max-pid-output",
		"max-receive-output",
		"correction-frequency-hz"
	};
	*/
};

#endif //_PID_CONFIG
