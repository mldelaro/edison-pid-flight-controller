#ifndef _PID_CONFIG
#define _PID_CONFIG

#include "./Properties.hpp"
#include "../FCConstants.hpp"

class PidConfig {
public:
	PidConfig();
	PidConfig(Properties* propertyFile);
	void setIsImuDataLabelled(bool enabled);
	double* getProportionalGain();
	double* getIntegralGain();
	double* getDerivativeGain();
	double* getMaxPidOutput();
	double* getAccTrim();
	int getMaxReceiverOutput();
	int getMaxThrottle();
	int getHoveringBaselineThrottle();
	int getCorrectionFrequencyHz();
	bool isUploadToS3EnabledOnStop();
	bool isUploadToTrainingDirEnabled();
	bool isUploadToS3ClassificationDirEnabled();
	bool isUploadToMarDatabaseEnabled();
	bool isImuDataLabelled();

private:
	Properties* propertyFile;
	double PGain[3];
	double IGain[3];
	double DGain[3];
	double PIDMaxOutput[3];
	double AccTrim[2];
	int maxReceiverOutput;
	int maxThrottle;
	int hoveringBaselineThrottle;
	int correctionFrequencyHz;
	bool uploadToS3TrainingDir;
	bool uploadToS3ClassificationDir;
	bool labelImuData;
	bool uploadToS3OnStop;
	bool uploadToMarDatabase;

	void _getDoubleValue(double* val, std::string key) {
		double valueFromProperty;
		valueFromProperty = propertyFile->getDoubleForKey(key);
		*val = valueFromProperty;
	}

	void _getIntValue(int* val, std::string key) {
		double valueFromProperty;
		valueFromProperty = propertyFile->getIntForKey(key);
		*val = valueFromProperty;
	}
};

#endif //_PID_CONFIG
