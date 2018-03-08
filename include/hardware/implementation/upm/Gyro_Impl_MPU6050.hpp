#include <mpu9250.hpp>
#include <mpu9150.hpp>
#include <mpu60x0.hpp>
#include <ak8975.hpp>
#include <mraa.hpp>

#include "./../../interface/Gyro_Interface.hpp"

#ifndef GYRO_IMPL_6050_H_
#define GYRO_IMPL_6050_H_

const float MPU6050_FS_SEL_0_GYRO_LSB = 131;
const float MPU6050_FS_SEL_1_GYRO_LSB = 65.5;
const float MPU6050_FS_SEL_2_GYRO_LSB = 32.8;
const float MPU6050_FS_SEL_3_GYRO_LSB = 16.4;

const float MPU6050_FS_SEL_0_ACC_LSB = 16384;
const float MPU6050_FS_SEL_1_ACC_LSB = 8192;
const float MPU6050_FS_SEL_2_ACC_LSB = 4096;
const float MPU6050_FS_SEL_3_ACC_LSB = 2048;


class Gyro_Impl_MPU6050 : public Gyro_Interface {
private:
	upm::MPU60X0* mpu6050;
	float acc_x;
	float acc_y;
	float acc_z;

	float gyro_x;
	float gyro_y;
	float gyro_z;

	int temp;

	bool isCallibrated;

	float gyro_lsbPerDps;
	float acc_lsbPerG;
	double gyro_sensorOffset[3];

	double acc_x_G;
	double acc_y_G;
	double acc_z_G;

	double gyro_x_DPS;
	double gyro_y_DPS;
	double gyro_z_DPS;

public:
	Gyro_Impl_MPU6050(int i2c_address) : Gyro_Interface(i2c_address) {
		std::cout << "MPU6050 - Constructor" << std::endl;
		acc_x = 0;
		acc_y = 0;
		acc_z = 0;
		gyro_x = 0;
		gyro_y = 0;
		gyro_z = 0;
		acc_x_G = 0;
		acc_y_G = 0;
		acc_z_G = 0;
		gyro_x_DPS = 0;
		gyro_y_DPS = 0;
		gyro_z_DPS = 0;
		temp = 0;
		isCallibrated = false;
		gyro_lsbPerDps = 0;
		acc_lsbPerG = 0;

		mpu6050 = new upm::MPU60X0(MPU60X0_I2C_BUS, MPU60X0_DEFAULT_I2C_ADDR);
	}

	void init() {
		bool didSucceed = false;
		while(!didSucceed) {
			didSucceed = true;
			std::cout << "MPU6050 - INIT via UPM" << std::endl;

			// Activate MPU6050
			if(mpu6050->init()) {
				usleep(15);
				// Set Register for 500dps @ full scale
				if(mpu6050->setGyroscopeScale(upm::MPU60X0::FS_SEL_T::FS_500)) {
					std::cout << "Set gyro scale to 500dps" << std::endl;
				} else {
					didSucceed = false;
				}
				// Set Register for +/- 8g @ full scale
				if(mpu6050->setAccelerometerScale(upm::MPU60X0::AFS_SEL_T::AFS_8)) {
					std::cout << "Set accelerometer scale to +/-8 G's" << std::endl;
				} else {
					didSucceed = false;
				}
				// Set Register for Digital Low Pass Filter
				if(mpu6050->setDigitalLowPassFilter(upm::MPU60X0::DLPF_44_42)) {
					std::cout << "Set Low pass filter" << std::endl;
				} else {
					didSucceed = false;
				}
				usleep(15);
			} else {
				std::cout << "Failed to initialize GYRO_MPU6050" << std::endl;
			}

			gyro_lsbPerDps = MPU6050_FS_SEL_1_GYRO_LSB;
			acc_lsbPerG = MPU6050_FS_SEL_1_ACC_LSB;
		}
	}

	void callibrateGyro(int sampleCount) {
		gyro_sensorOffset[0] = 0;
		gyro_sensorOffset[1] = 0;
		gyro_sensorOffset[2] = 0;

		acc_x_G = 0;
		acc_y_G = 0;
		acc_z_G = 0;
		gyro_x_DPS = 0;
		gyro_y_DPS = 0;
		gyro_z_DPS = 0;

		isCallibrated = false;
		for(int currentSampleIndex = 0; currentSampleIndex < sampleCount; currentSampleIndex++) {
			read();
			usleep(30);
			mpu6050->getGyroscope(&gyro_x, &gyro_y, &gyro_z);
			gyro_sensorOffset[0] += gyro_x;
			gyro_sensorOffset[1] += gyro_y;
			gyro_sensorOffset[2] += gyro_z;
			usleep(30);
		}
		isCallibrated = true;
		gyro_sensorOffset[0] /= (double)sampleCount;
		gyro_sensorOffset[1] /= (double)sampleCount;
		gyro_sensorOffset[2] /= (double)sampleCount;

		std::cout << "Callibrated Gyro Roll: " << gyro_sensorOffset[0] << std::endl;
		std::cout << "Callibrated Gyro Pitch: " << gyro_sensorOffset[1] << std::endl;
		std::cout << "Callibrated Gyro Yaw: " << gyro_sensorOffset[2] << std::endl;
	}

	void read() {
		mpu6050->update();
		mpu6050->getGyroscope(&gyro_x, &gyro_y, &gyro_z);
		mpu6050->getAccelerometer(&acc_x, &acc_y, &acc_z);

		gyro_x_DPS = -gyro_x + gyro_sensorOffset[0];
		gyro_y_DPS = gyro_y - gyro_sensorOffset[1];
		gyro_z_DPS = -gyro_z + gyro_sensorOffset[2];

		acc_x_G = acc_y;
		acc_y_G = -acc_x;
		acc_z_G = -acc_z;
	}

	double getRoll_DPS () {
		return gyro_x_DPS;
	}

	double getPitch_DPS () {
		return gyro_y_DPS;
	}

	double getYaw_DPS () {
		return gyro_z_DPS;
	}

	double getAccX_G () {
		return acc_x_G;
	}

	double getAccY_G () {
		return acc_y_G;
	}

	double getAccZ_G () {
		return acc_z_G;
	}

	~Gyro_Impl_MPU6050() {
		std::cout << "MPU6050 - DESTROY" << std::endl;
	}

};

#endif // GYRO_IMPL_6050_H_
