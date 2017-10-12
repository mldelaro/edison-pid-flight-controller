#include "../../include/utility/PidConfig.hpp"

PidConfig::PidConfig() {
	propertyFile = NULL;
	PGain[0] = fc_constants::PID_PROPORTIONAL_GAIN[0];
	PGain[1] = fc_constants::PID_PROPORTIONAL_GAIN[1];
	PGain[2] = fc_constants::PID_PROPORTIONAL_GAIN[2];

	IGain[0] = fc_constants::PID_INTEGRAL_GAIN[0];
	IGain[1] = fc_constants::PID_INTEGRAL_GAIN[1];
	IGain[2] = fc_constants::PID_INTEGRAL_GAIN[2];

	DGain[0] = fc_constants::PID_DERIVATIVE_GAIN[0];
	DGain[1] = fc_constants::PID_DERIVATIVE_GAIN[1];
	DGain[2] = fc_constants::PID_DERIVATIVE_GAIN[2];

	PIDMaxOutput[0] = fc_constants::PID_MAX_OUTPUT[0];
	PIDMaxOutput[1] = fc_constants::PID_MAX_OUTPUT[1];
	PIDMaxOutput[2] = fc_constants::PID_MAX_OUTPUT[2];

	maxReceiverOutput = fc_constants::MAX_RECEIVER_THROTTLE;
	maxThrottle = 2300;
	hoveringBaselineThrottle = 1700;
	correctionFrequencyHz = fc_constants::CORRECTION_FREQUENCY_HZ;

	csvPidOutputEnabled = false;
	csvRawGyroOutputEnabled = false;
}

PidConfig::PidConfig(Properties* propertiesFile) {
	propertyFile = propertiesFile;

	PGain[0] = fc_constants::PID_PROPORTIONAL_GAIN[0];
	PGain[1] = fc_constants::PID_PROPORTIONAL_GAIN[1];
	PGain[2] = fc_constants::PID_PROPORTIONAL_GAIN[2];

	IGain[0] = fc_constants::PID_INTEGRAL_GAIN[0];
	IGain[1] = fc_constants::PID_INTEGRAL_GAIN[1];
	IGain[2] = fc_constants::PID_INTEGRAL_GAIN[2];

	DGain[0] = fc_constants::PID_DERIVATIVE_GAIN[0];
	DGain[1] = fc_constants::PID_DERIVATIVE_GAIN[1];
	DGain[2] = fc_constants::PID_DERIVATIVE_GAIN[2];

	PIDMaxOutput[0] = fc_constants::PID_MAX_OUTPUT[0];
	PIDMaxOutput[1] = fc_constants::PID_MAX_OUTPUT[1];
	PIDMaxOutput[2] = fc_constants::PID_MAX_OUTPUT[2];

	maxReceiverOutput = fc_constants::MAX_RECEIVER_THROTTLE;
	maxThrottle = 2300;
	hoveringBaselineThrottle = 1700;
	correctionFrequencyHz = fc_constants::CORRECTION_FREQUENCY_HZ;

	_getDoubleValue(&PGain[0], "p-gain-roll");
	_getDoubleValue(&PGain[1], "p-gain-pitch");
	_getDoubleValue(&PGain[2], "p-gain-yaw");

	_getDoubleValue(&IGain[0], "i-gain-roll");
	_getDoubleValue(&IGain[1], "i-gain-pitch");
	_getDoubleValue(&IGain[2], "i-gain-yaw");

	_getDoubleValue(&DGain[0], "d-gain-roll");
	_getDoubleValue(&DGain[1], "d-gain-pitch");
	_getDoubleValue(&DGain[2], "d-gain-yaw");

	_getDoubleValue(&PIDMaxOutput[0], "pid-max-output-roll");
	_getDoubleValue(&PIDMaxOutput[1], "pid-max-output-pitch");
	_getDoubleValue(&PIDMaxOutput[2], "pid-max-output-yaw");

	_getIntValue(&maxReceiverOutput, "max-receive-output");
	_getIntValue(&maxThrottle, "max-throttle");
	_getIntValue(&hoveringBaselineThrottle, "hovering-baseline-throttle");
	_getIntValue(&correctionFrequencyHz, "correction-frequency-hz");

	csvPidOutputEnabled = propertyFile->getBoolForKey("enable-pid-output-csv-log");
	csvRawGyroOutputEnabled = propertyFile->getBoolForKey("enable-raw-gyro-csv-log");
}

double* PidConfig::getProportionalGain() {
	return PGain;
}
double* PidConfig::getIntegralGain() {
	return IGain;
}
double* PidConfig::getDerivativeGain() {
	return DGain;
}
double* PidConfig::getMaxPidOutput() {
	return PIDMaxOutput;
}
int PidConfig::getMaxThrottle() {
	return maxThrottle;
}
int PidConfig::getHoveringBaselineThrottle() {
	return hoveringBaselineThrottle;
}
int PidConfig::getMaxReceiverOutput() {
	return maxReceiverOutput;
}
int PidConfig::getCorrectionFrequencyHz() {
	return correctionFrequencyHz;
}
bool PidConfig::isCsvPidOutputEnabled() {
	return csvPidOutputEnabled;
}
bool PidConfig::isCsvRawGyroOutputEnabled() {
	return csvRawGyroOutputEnabled;
}
