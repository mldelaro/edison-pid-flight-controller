#include "../include/PidController.hpp"

bool enable_gyroLog = true;

double gyro_sensorOffset[3];		// gyro callibration
double gyro_sensorNormalized[3];	// callibrated gyro sensor input
double gyro_sensorPidInput[3];		// error rate detection from gyro as input to PID

double degreeTraveledPerSample;		// ratio for converting sensor input to degrees traveled within a set loop frequency
float currentAngle[2];
float angleTraveled[2];				// traveled degrees

float accVector;
float accAngleMeasuredFromNormalG[3];

float autoLevelSelfAdjust[2];

int batteryVoltage;

double pidRunningError[3];
double pidSetPoint[3];
double pidIntegralMemory[3];
double pidLastDifferentialError[3];
double pidOutput[3];

double pidRunningBaselineThrottle;
double pidRunningThrottle[4];

double remainingLoopTimeout;
double pidRunningTime;
double loopTimeout;
double loopRunningTime;
double averageLoopRunningTime;
double averageLoopFrequency;
int loopCounter;

mraa::Aio* batteryVoltageAnalogInput;

FCLogger* CsvLoggerPidOutput;
FCLogger* CsvLoggerGyroRawOutput;
FCLogger* PidControlLogger;
FCLogger* PidErrorLogger;

std::mutex mutex_pilot_mem;

boost::interprocess::shared_memory_object* shared_mem_tcp_receiver;
//boost::interprocess::mapped_region* region_tcp_receiver;
std::stringstream fcReceiver;
std::string rx_tcp_server; // string read from tcp server's shared memory

PidController::~PidController() {

	esc_controller->setPwmCycle(0, 0, 1000);
	esc_controller->setPwmCycle(1, 0, 1000);
	esc_controller->setPwmCycle(2, 0, 1000);
	esc_controller->setPwmCycle(3, 0, 1000);

	batteryVoltageAnalogInput->~Aio();
	batteryVoltageAnalogInput = NULL;

	gyro->~I2C_Slave_Device();
	gyro = NULL;
	esc_controller->~I2C_Slave_Device();
	esc_controller = NULL;

	CsvLoggerPidOutput->~FCLogger();
	CsvLoggerPidOutput = NULL;
	CsvLoggerGyroRawOutput->~FCLogger();
	CsvLoggerGyroRawOutput = NULL;
	PidControlLogger->~FCLogger();
	PidControlLogger = NULL;
	PidErrorLogger->~FCLogger();
	PidErrorLogger = NULL;
}


PidController::PidController(PidConfig* config) {

	if(config == NULL) {
		std::cout << "Running on default PID Configurations..." << std::endl;
		pidConfigs = new PidConfig();
	} else {
		std::cout << "Running on custom PID Configurations..." << std::endl;
		pidConfigs = config;
	}

	if(pidConfigs->isCsvRawGyroOutputEnabled()) {
		const char* sensorCsvColumnLabels[6];
		sensorCsvColumnLabels[0] = "accelerometer_x";
		sensorCsvColumnLabels[1] = "accelerometer_y";
		sensorCsvColumnLabels[2] = "accelerometer_z";
		sensorCsvColumnLabels[3] = "gyrometer_x";
		sensorCsvColumnLabels[4] = "gyrometer_y";
		sensorCsvColumnLabels[5] = "gyrometer_z";

		CsvLoggerGyroRawOutput = new FCLogger("/home/root/logging/csv-stats/", "RAW_Gyro", ".csv");
		CsvLoggerGyroRawOutput->startCsvLog(6, sensorCsvColumnLabels);
	} else {
		std::cout << "[DISABLED] Raw Gyro CSV data log..." << std::endl;
	}

	if(pidConfigs->isCsvPidOutputEnabled()) {
		const char* gyroCsvColumnLabels[9];
		//gyroCsvColumnLabels[0] = "time_usec";
		gyroCsvColumnLabels[0] = "runtime";
		gyroCsvColumnLabels[1] = "callibrated_gyro_roll";
		gyroCsvColumnLabels[2] = "callibrated_gyro_pitch";
		gyroCsvColumnLabels[3] = "callibrated_yaw";
		gyroCsvColumnLabels[4] = "degree_traveled_roll";
		gyroCsvColumnLabels[5] = "degree_traveled_pitch";
		gyroCsvColumnLabels[6] = "pid_output_roll";
		gyroCsvColumnLabels[7] = "pid_output_pitch";
		gyroCsvColumnLabels[8] = "pid_output_yaw";

		CsvLoggerPidOutput = new FCLogger("/home/root/logging/csv-stats/", "PID_Output", ".csv");
		CsvLoggerPidOutput->startCsvLog(9, gyroCsvColumnLabels);
	} else {
		std::cout << "[DISABLED] PID output CSV data log..." << std::endl;
	}

	PidControlLogger = new FCLogger("/home/root/logging/pid-controller/", "PidControllerLog", ".txt");
	PidErrorLogger = new FCLogger("/home/root/logging/pid-controller/", "PidErrorLog", ".txt");

	mraa::init();

	i2c_controller = NULL;
	i2c_slaveDevices = NULL;
	gyro = NULL;
	esc_controller = NULL;
	currentStatus = INIT;

	PidControlLogger->logStream << "Initializing PID Flight Controller..." << std::endl;
	PidControlLogger->logStream << "Initializing Analog and GPIO pins..." << std::endl;

	batteryVoltageAnalogInput = new mraa::Aio(fc_constants::PIN_BATTERY_VOLTAGE);
	if(batteryVoltageAnalogInput == NULL) {
		PidErrorLogger->logStream << "WARNING: Cannot initialize analog input";
	}

	PidControlLogger->logStream << "Initializing I2C Devices..." << std::endl;
	mraa::I2c* i2c_controller = new mraa::I2c(0);

	mraa::Result setI2cFreqReturnCode = i2c_controller->frequency(mraa::I2cMode::I2C_FAST); // operate in standard 100kHz mode
	if(setI2cFreqReturnCode != mraa::Result::SUCCESS) {
		PidControlLogger->logStream << "Failed to initialize I2C Bus in Fast mode..." << std::endl;
		std::cout << "Failed to initialize I2C Bus in Fast mode..." << std::endl;
	} else {
		std::cout << "Initialized I2C in Fast mode" << std::endl;
	}

	i2c_slaveDevices = new I2C_Slave_Device*[fc_constants::I2C_DEVICE_COUNT];

	PidControlLogger->logStream << "Constructing Gyro..." << std::endl;
	gyro = new Gyro_Impl_MPU6050(fc_constants::I2C_ADDR_GYRO, i2c_controller);
	gyro->init();
	i2c_slaveDevices[0] = gyro;
	usleep(100);

	/* isntances interfere with each other?*/
	PidControlLogger->logStream << "Constructing ESC Controller..." << std::endl;
	esc_controller = new ESC_Impl_PCA9685(fc_constants::I2C_ADDR_ESC_CONTROLLER, i2c_controller, 4);
	esc_controller->init();
	i2c_slaveDevices[1] = esc_controller;
	usleep(100);

	esc_controller->setPwmFrequency(300);
	esc_controller->setPwmCycle(0, 0, 1000);
	esc_controller->setPwmCycle(1, 0, 1000);
	esc_controller->setPwmCycle(2, 0, 1000);
	esc_controller->setPwmCycle(3, 0, 1000);

	PidControlLogger->logStream << "Initializing runtime variables..." << std::endl;
	degreeTraveledPerSample = (double)((1.0) / (pidConfigs->getCorrectionFrequencyHz() * fc_constants::GYRO_1DPS_RAW_OUTPUT));
	PidControlLogger->logStream << "- Degrees traveled per sample [.00006] = " << degreeTraveledPerSample << std::endl;
	loopTimeout = (1.0 / pidConfigs->getCorrectionFrequencyHz()) * 1000000;
	PidControlLogger->logStream << "- Loop Timeout [useconds] = " << loopTimeout << std::endl;
}

void PidController::_TEST_ROTORS() {
	std::cout << "RUNNING TEST"<< std::endl;
	//esc_controller->setPwmFrequency(300);
	sleep(1);


	/* Throttle test
	int i = 1100;
	while(i < 2300) {
		esc_controller->setPwmCycle(0, 0, i);
		std::cout << "STARTUP_" << i << "..."<< std::endl;
		esc_controller->setPwmCycle(0, 0, i);
		i += 50;
		sleep(1);
	}
	*/

	for(int i = 0; i < 4; i++) {
		esc_controller->setPwmCycle(i, 0, 1100);
		std::cout << "STARTUP_" << i << "..."<< std::endl;
		esc_controller->setPwmCycle(i, 0, 1100);
		sleep(1);

		esc_controller->setPwmCycle(i, 0, 1300);
		std::cout << "LOW_THROTTLE_" << i << "..."<< std::endl;
		esc_controller->setPwmCycle(i, 0, 1300);
		sleep(2);

		esc_controller->setPwmCycle(i, 0, 1000);
		std::cout << "STOP_" << i << "..."<< std::endl;
		esc_controller->setPwmCycle(i, 0, 1000);
		sleep(1);
	}
}

void PidController::_STOP() {
	esc_controller->setPwmCycle(0, 0, 800);
	esc_controller->setPwmCycle(1, 0, 800);
	esc_controller->setPwmCycle(2, 0, 800);
	esc_controller->setPwmCycle(3, 0, 800);
}

void PidController::setup() {
	PidControlLogger->logStream << "ENTER SETUP" << std::endl;

	currentStatus = SETUP;
	loopCounter = 0;

	// keep hover status... set all setpoints to 0
	pidSetPoint[ROLL] = 0;
	pidSetPoint[PITCH] = 0;
	pidSetPoint[YAW] = 0;

	// Normalize gyro and accelerometer readings
	PidControlLogger->logStream << "Callibrating Gyro Device..." << std::endl;
	currentStatus = GYRO_CALLIBRATION;

	for(int currentSampleIndex = 0; currentSampleIndex < fc_constants::GYRO_SAMPLE_COUNT; currentSampleIndex++) {
		gyro->read();

		gyro_sensorOffset[ROLL] += gyro->getGyro_roll();
		gyro_sensorOffset[PITCH] += gyro->getGyro_pitch();
		gyro_sensorOffset[YAW] += gyro->getGyro_yaw();

		usleep(15);
	}

	gyro_sensorOffset[ROLL] /= fc_constants::GYRO_SAMPLE_COUNT;
	gyro_sensorOffset[PITCH] /= fc_constants::GYRO_SAMPLE_COUNT;
	gyro_sensorOffset[YAW] /= fc_constants::GYRO_SAMPLE_COUNT;

	PidControlLogger->logStream << "Finished Gyro Callibration..." << std::endl;
	currentStatus = START_MOTORS;

	PidControlLogger->logStream << "EXIT SETUP " << std::endl;
}

void PidController::iterativeLoop(bool rotorsEnabled) {
	loop(rotorsEnabled);
}

void PidController::loop(bool rotorsEnabled) {
	auto loopStartTime = std::chrono::high_resolution_clock::now();

	/* BASIC IMU Calculations */
	// Normalize gyro raw data to calibration
	gyro_sensorNormalized[ROLL] = gyro->getGyro_roll() - gyro_sensorOffset[ROLL];
	gyro_sensorNormalized[PITCH] = gyro->getGyro_pitch() - gyro_sensorOffset[PITCH];
	gyro_sensorNormalized[YAW] = gyro->getGyro_yaw() - gyro_sensorOffset[YAW];

	gyro_sensorNormalized[YAW] *= -1; // rotate yaw to match conventions

	gyro_sensorPidInput[ROLL] = (gyro_sensorPidInput[ROLL] * 0.7) + ((gyro_sensorNormalized[ROLL] / fc_constants::GYRO_1DPS_RAW_OUTPUT) * 0.3);
	gyro_sensorPidInput[PITCH] = (gyro_sensorPidInput[PITCH] * 0.7) + ((gyro_sensorNormalized[PITCH] / fc_constants::GYRO_1DPS_RAW_OUTPUT) * 0.3);
	gyro_sensorPidInput[YAW] = (gyro_sensorPidInput[YAW] * 0.7) + ((gyro_sensorNormalized[YAW] / fc_constants::GYRO_1DPS_RAW_OUTPUT) * 0.3);

	// Calculate Angle Traveled
	//degreeTraveledPerSample = (double)((double)(1.0) / (double)fc_constants::CORRECTION_FREQUENCY_HZ / (double)fc_constants::GYRO_1DPS_RAW_OUTPUT);
	//degreeTraveledPerSample *= (double)((double)(loopRunningTime / 10) * (double)(1.0 / fc_constants::CORRECTION_FREQUENCY_HZ));
	angleTraveled[PITCH] += gyro_sensorNormalized[PITCH] * degreeTraveledPerSample;
	angleTraveled[ROLL] += gyro_sensorNormalized[ROLL] * degreeTraveledPerSample;

	// Translate changes in yaw to traveled angles in pitch and roll
	angleTraveled[PITCH] += angleTraveled[ROLL] * sin(gyro_sensorNormalized[YAW] * fc_constants::RATIO_DEGREE_TO_RADIAN);
	angleTraveled[ROLL] -= angleTraveled[PITCH] * sin(gyro_sensorNormalized[YAW] * fc_constants::RATIO_DEGREE_TO_RADIAN);

	/* Accelerometer angle calculations*/
	accVector = sqrt(
						(gyro->getAcc_x()*gyro->getAcc_x()) +
						(gyro->getAcc_y()*gyro->getAcc_y()) +
						(gyro->getAcc_z()*gyro->getAcc_z())
					);

	// Declare pitch and roll angle w.r.t gravitational pull
	if(abs(gyro->getAcc_x()) < accVector) {
		accAngleMeasuredFromNormalG[ROLL] = asin((double)gyro->getAcc_x()/accVector) * -fc_constants::RATIO_RADIAN_TO_DEGREE;
	}

	if(abs(gyro->getAcc_y()) < accVector) {
		accAngleMeasuredFromNormalG[PITCH] = asin((double)gyro->getAcc_y()/accVector) * fc_constants::RATIO_RADIAN_TO_DEGREE;
	}

	// Trim Acc calibration
	//accAngleMeasuredFromNormalG[PITCH] -= 0.7;
	accAngleMeasuredFromNormalG[ROLL] += 1.1;


	// Drift compensation
	angleTraveled[PITCH] = angleTraveled[PITCH]*0.9996 + accAngleMeasuredFromNormalG[PITCH] * 0.0004;
	angleTraveled[ROLL] = angleTraveled[ROLL]*0.9996 + accAngleMeasuredFromNormalG[ROLL] * 0.0004;

	autoLevelSelfAdjust[PITCH] = angleTraveled[PITCH] * 15;
	autoLevelSelfAdjust[ROLL] = angleTraveled[ROLL] * 15;

	/* Read from TCP Server's shared memory
	if(region_tcp_receiver != NULL) {
		std::stringstream stream;
		stream.rdbuf()->pubsetbuf((char*)region_tcp_receiver->get_address(), region_tcp_receiver->get_size());
		rx_tcp_server = stream.str();
	} else {

	}*/

	/* Start takeoff */
	if(currentStatus == START_MOTORS) {
		pidRunningBaselineThrottle = pidConfigs->getHoveringBaselineThrottle();
		pidSetPoint[ROLL] = 0;
		pidSetPoint[PITCH] = 0;
		pidSetPoint[YAW] = 0;

		angleTraveled[PITCH] = accAngleMeasuredFromNormalG[PITCH];
		angleTraveled[ROLL] = accAngleMeasuredFromNormalG[ROLL];

		//Reset PID Controller
		pidIntegralMemory[0] = 0;
		pidIntegralMemory[1] = 0;
		pidIntegralMemory[2] = 0;
		pidLastDifferentialError[0] = 0;
		pidLastDifferentialError[1] = 0;
		pidLastDifferentialError[2] = 0;

		currentStatus = RUNNING;
	}

	gyro->read();
	calculatePidController();

	if(currentStatus == RUNNING) {
		// limit throttle to be able to compensate for max feedback from PID controller
		if(pidRunningBaselineThrottle > pidConfigs->getMaxReceiverOutput()) {
			pidRunningBaselineThrottle = pidConfigs->getMaxReceiverOutput();
		}

		// ROTOR 1 - Clockwise 			@ front left
		pidRunningThrottle[0] = pidRunningBaselineThrottle - pidOutput[PITCH] - pidOutput[ROLL] + pidOutput[YAW];

		// ROTOR 2 - Counter-Clockwise 	@ front right
		pidRunningThrottle[1] = pidRunningBaselineThrottle - pidOutput[PITCH] + pidOutput[ROLL] - pidOutput[YAW];

		// ROTOR 3 - Counter-Clockwise 	@ rear left
		pidRunningThrottle[2] = pidRunningBaselineThrottle + pidOutput[PITCH] - pidOutput[ROLL] - pidOutput[YAW];

		// ROTOR 4 - Clockwise			@ rear right
		pidRunningThrottle[3] = pidRunningBaselineThrottle + pidOutput[PITCH] + pidOutput[ROLL] + pidOutput[YAW];

	} else {
		// feed ESC 1000us pulses
		pidRunningThrottle[0] = 1000;
		pidRunningThrottle[1] = 1000;
		pidRunningThrottle[2] = 1000;
		pidRunningThrottle[3] = 1000;
	}

	// SET THROTTLE FOR EACH ROTOR
	if(rotorsEnabled) {
		// limit esc pulse
		for(int i = 0; i < 4; i++) {
			if(pidRunningThrottle[i] > pidConfigs->getMaxThrottle()) {
				pidRunningThrottle[i] = pidConfigs->getMaxThrottle();
			}
		}

		esc_controller->setPwmCycle(0, 0, pidRunningThrottle[0]);
		esc_controller->setPwmCycle(1, 0, pidRunningThrottle[1]);
		esc_controller->setPwmCycle(2, 0, pidRunningThrottle[2]);
		esc_controller->setPwmCycle(3, 0, pidRunningThrottle[3]);
	}

	loopCounter++;

	auto loopEndTime = std::chrono::high_resolution_clock::now();
	auto elapsedLoopTime = std::chrono::duration_cast<std::chrono::microseconds>(loopEndTime - loopStartTime);
	loopRunningTime = elapsedLoopTime.count();
	remainingLoopTimeout = loopTimeout - loopRunningTime;

	//Get total running time
	//auto totalElapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(loopEndTime - pidStartTime);
	//pidRunningTime = totalElapsedTime.count();

	if(pidConfigs->isCsvPidOutputEnabled()) {
		CsvLoggerPidOutput->logStream
			<< pidRunningTime << ","
			<< gyro_sensorNormalized[ROLL] << ","
			<< gyro_sensorNormalized[PITCH] << ","
			<< gyro_sensorNormalized[YAW] << ","
			<< angleTraveled[ROLL] << ","
			<< angleTraveled[PITCH] << ","
			<< pidOutput[ROLL] << ","
			<< pidOutput[PITCH] << ","
			<< pidOutput[YAW] << std::endl;
	}

	if(pidConfigs->isCsvRawGyroOutputEnabled()) {
		CsvLoggerGyroRawOutput->logStream
			<< gyro->getAcc_x() << ","
			<< gyro->getAcc_y() << ","
			<< gyro->getAcc_z() << ","
			<< gyro_sensorNormalized[ROLL] << ","
			<< gyro_sensorNormalized[PITCH] << ","
			<< gyro_sensorNormalized[YAW] << std::endl;
	}


	if((int)(loopCounter % (1 * pidConfigs->getCorrectionFrequencyHz())) == (int)(0)) {
		if(!rotorsEnabled) {
//			std::cout << "TCP Receive: " << rx_tcp_server << std::endl;

//			std::cout << "Running Baseline: " << pidRunningBaselineThrottle << std::endl;

//			std::cout << "Running Time: " << remainingLoopTimeout << std::endl;
//			std::cout << "Roll Traveled: " << angleTraveled[ROLL] << std::endl;
//			std::cout << "Pitch Traveled: " << angleTraveled[PITCH] << std::endl;

			// for use with accelerometer callibration [helps trim acc value]
//			std::cout << "AccPitch: " << accAngleMeasuredFromNormalG[PITCH] << std::endl;
//			std::cout << "AccRoll: " << accAngleMeasuredFromNormalG[ROLL] << std::endl;

//			std::cout << "Roll Gyro Output: " << gyro_sensorNormalized[ROLL] << std::endl;
//			std::cout << "Pitch Gyro Ouptut: " << gyro_sensorNormalized[PITCH] << std::endl;
			std::cout << "Yaw Gyro Ouptut: " << gyro_sensorNormalized[YAW] << std::endl;

		}

//			std::cout << "Roll RunningError: " << pidRunningError[ROLL] << std::endl;
//			std::cout << "Pitch RunningError: " << pidRunningError[PITCH] << std::endl;
//			std::cout << "Yaw RunningError: " << pidRunningError[YAW] << std::endl;

//			std::cout << "Roll PID Output: " << pidOutput[ROLL] << std::endl;
//			std::cout << "Pitch PID Ouptut: " << pidOutput[PITCH] << std::endl;
//			std::cout << "Yaw PID Ouptut: " << pidOutput[YAW] << std::endl;

//			std::cout << "ROTOR 1 Throttle: " << pidRunningThrottle[0] << std::endl;
//			std::cout << "ROTOR 2 Throttle: " << pidRunningThrottle[1] << std::endl;
//			std::cout << "ROTOR 3 Throttle: " << pidRunningThrottle[2] << std::endl;
//			std::cout << "ROTOR 4 Throttle: " << pidRunningThrottle[3] << std::endl;

	}

	if(remainingLoopTimeout < 100) {
		PidErrorLogger->logStream << "Running slow by " << remainingLoopTimeout << std::endl;
	} else {
		usleep(remainingLoopTimeout - 100);
	}
}



void PidController::incrementBaselineThrottle(int value) {
	if(value < 0) {
		if(pidRunningBaselineThrottle > 1600) {
			pidRunningBaselineThrottle = pidRunningBaselineThrottle + value;
		} else {
			pidRunningBaselineThrottle = 1601;
		}
	}

	if(value > 0) {
		if (pidRunningBaselineThrottle < 2000) {
			pidRunningBaselineThrottle = pidRunningBaselineThrottle + value;
		} else {
			pidRunningBaselineThrottle = 1999;
		}
	}



}

void PidController::setRollDPS(int dps) {
	pidSetPoint[ROLL] = dps;
}

void PidController::setPitchDPS(int dps) {
	pidSetPoint[PITCH] = dps;

}

void PidController::setYawDPS(int dps) {
	pidSetPoint[YAW] = dps;

}


void PidController::calculatePidController() {
	// iterate PID Calculator for roll, pitch, and yaw
	for(int i = 0; i < 3; i++) {
		pidRunningError[i] = gyro_sensorPidInput[i] - pidSetPoint[i];
		pidIntegralMemory[i] += fc_constants::PID_INTEGRAL_GAIN[i] * pidRunningError[i];
		if(pidIntegralMemory[i] > fc_constants::PID_MAX_OUTPUT[i]) {
			pidIntegralMemory[i] = fc_constants::PID_MAX_OUTPUT[i];
		} else if (pidIntegralMemory[i] < (-1 * fc_constants::PID_MAX_OUTPUT[i])) {
			pidIntegralMemory[i] = (-1 * fc_constants::PID_MAX_OUTPUT[i]);
		}

		// Get PID-output
		pidOutput[i] = fc_constants::PID_PROPORTIONAL_GAIN[i] * pidRunningError[i] + pidIntegralMemory[i] + fc_constants::PID_DERIVATIVE_GAIN[i] * (pidRunningError[i] - pidLastDifferentialError[i]);
		pidLastDifferentialError[i] = pidRunningError[i];
	}
}

void* PidController::p_loop() {
	boost::interprocess::shared_memory_object shared_mem_pilot(
			boost::interprocess::open_only,
			"shared_mem_pilot",
			boost::interprocess::read_write
	);
	boost::interprocess::mapped_region region(shared_mem_pilot, boost::interprocess::read_write);
	std::stringstream stream;
	Properties* fcProperties = new Properties("/home/root/FlightController/flightController.properties");
	pidConfigs = new PidConfig(fcProperties);

	PidController* flightController = new PidController(pidConfigs);
	flightController->setup();

	bool didStartFlight = false;
	bool enabledRotors = false;

	while(true) {
		stream.rdbuf()->pubsetbuf((char*)region.get_address(), region.get_size());
		std::string rx_host = stream.str();

		if(rx_host.c_str()[0] != '\0') {
			std::memset(region.get_address(), '\0', region.get_size());

			if(strncmp(rx_host.c_str(), "T", 1) == 0) {
				std::cout << "Entering testing mode...\n";
				flightController->_TEST_ROTORS();
				didStartFlight = false;
			} else if(strncmp(rx_host.c_str(), "V", 1) == 0) {
				std::cout << "STARTING FLIGHT CONTROLLER...\n";
				flightController->iterativeLoop(true);
				enabledRotors = true;
				didStartFlight = true;
			} else if(strncmp(rx_host.c_str(), "Y", 1) == 0) {
				std::cout << "[FALSE]STARTING FLIGHT CONTROLLER...\n";
				flightController->_STOP();
				flightController->iterativeLoop(false);
				enabledRotors = false;
				didStartFlight = true;
			} else if(strncmp(rx_host.c_str(), "W", 1) == 0) {
				std::cout << "STOPPING FLIGHT CONTROLLER...\n";
				flightController->_STOP();
				didStartFlight = false;
				enabledRotors = false;
			} else if(strncmp(rx_host.c_str(), "S", 1) == 0) {
				if(didStartFlight) {
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(500000);
				}
			} else if(strncmp(rx_host.c_str(), "F", 1) == 0) {
				//TODO: go forward
				if(didStartFlight) {
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(250000);
				}
			} else if(strncmp(rx_host.c_str(), "B", 1) == 0) {
				//TODO: go backward
				if(didStartFlight) {
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(250000);
				}
			} else if(strncmp(rx_host.c_str(), "L", 1) == 0) {
				//TODO: go Left
				if(didStartFlight) {
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(250000);
				}
			} else if(strncmp(rx_host.c_str(), "R", 1) == 0) {
				//TODO: go Right
				if(didStartFlight) {
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(250000);
				}
			} else if(strncmp(rx_host.c_str(), "U", 1) == 0) {
				//TODO: go up
				if(didStartFlight) {
					if(pidRunningBaselineThrottle < 1900) {
						pidRunningBaselineThrottle += 2;
					}
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(250000);
				}
			} else if(strncmp(rx_host.c_str(), "D", 1) == 0) {
				//TODO: go down
				if(didStartFlight) {
					if(pidRunningBaselineThrottle > 1400) {
						pidRunningBaselineThrottle -= 5;
					}
					flightController->iterativeLoop(enabledRotors);
				} else {
					usleep(250000);
				}
			} else {
				std::cout << "Failed command: " << rx_host << " of length " << rx_host.length() << std::endl;
				sleep(3);
			}
			stream.str("");
			rx_host.clear();
		} else {
			if(didStartFlight) {
				flightController->iterativeLoop(enabledRotors);
			} else {
				usleep(500000);
			}
		}
	}
	return NULL;
}
