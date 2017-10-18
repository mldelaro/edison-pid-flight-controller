#ifndef GYRO_IMPL_6050_H_
#define GYRO_IMPL_6050_H_

#include <mraa.hpp>

#include "./../interface/Gyro_Interface.hpp"
#include "./../I2C_Slave_Device.hpp"

const int MPU6050_TX_RX_BUFFER_LENGTH = 14;
const int MPU6050_I2C_ADDRESS = 0x68;

const int MPU6050_REG_GYRO_CONFIG = 0x1B;
const int MPU6050_REG_ACCEL_CONFIG = 0x1C;

const int MPU6050_FS_SEL_0 = 0x00;
const int MPU6050_FS_SEL_1 = 0x08;
const int MPU6050_FS_SEL_2 = 0x10;
const int MPU6050_FS_SEL_3 = 0x18;

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

	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int temp;

	bool isCallibrated;

	float gyro_lsbPerDps;
	float acc_lsbPerG;
	float gyro_sensorOffset[3];

	double acc_x_G;
	double acc_y_G;
	double acc_z_G;

	double gyro_x_DPS;
	double gyro_y_DPS;
	double gyro_z_DPS;

public:
	Gyro_Impl_MPU6050(int i2c_address, mraa::I2c* i2c_controller) : Gyro_Interface(i2c_address, i2c_controller) {
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
	}

	void init() {
		bool didSucceed = false;
		while(!didSucceed) {
			std::cout << "MPU6050 - INIT" << std::endl;

			// Write to PWR_MGMT_1 Register - 0x6B
			// Activate gyro by writing 0 to register bits
			writeByteToAddress(0x6B, 0x00);

			// Write to GYRO_CONFIG Register - 0x1B
			// Set Register for 500dps @ full scale
			writeByteToAddress(MPU6050_REG_GYRO_CONFIG, MPU6050_FS_SEL_1);
			gyro_lsbPerDps = MPU6050_FS_SEL_1_GYRO_LSB;

			// Write to ACCEL_CONFIG Register - 0x1C
			// Set Register for +/- 8g @ full scale
			writeByteToAddress(MPU6050_REG_ACCEL_CONFIG, MPU6050_FS_SEL_1);
			acc_lsbPerG = MPU6050_FS_SEL_1_ACC_LSB;

			// Write to CONFIG Register - 0x1A
			// Set Register for Digital Low Pass Filter
			writeByteToAddress(0x1A, 0x03);

			// SELF-CHECK: read back configurations
			// Read from Register 0x1B
			if(readByteValueFromAddress(MPU6050_REG_ACCEL_CONFIG) != MPU6050_FS_SEL_1) {
				std::cout << "Failed to initialize GYRO_MPU6050" << std::endl;
				std::cout << "Resetting..." << std::endl;
			} else {
				didSucceed = true;
			}
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
			usleep(15);
			gyro_sensorOffset[0] += gyro_x;
			gyro_sensorOffset[1] += gyro_y;
			gyro_sensorOffset[2] += gyro_z;
		}
		isCallibrated = true;
		gyro_sensorOffset[0] /= sampleCount;
		gyro_sensorOffset[1] /= sampleCount;
		gyro_sensorOffset[2] /= sampleCount;

		std::cout << "Callibrated Gyro Roll: " << gyro_sensorOffset[0] << std::endl;
		std::cout << "Callibrated Gyro Pitch: " << gyro_sensorOffset[1] << std::endl;
		std::cout << "Callibrated Gyro Yaw: " << gyro_sensorOffset[2] << std::endl;
	}

	void read() {
		i2c->address(i2c_address); // set talking slave (before each read/write)
		rx_tx_buf[0] = 0x3B;	// Read from Register 0x3B
		i2c->write(rx_tx_buf, 1);

		i2c->address(i2c_address); // set talking slave (before each read/write)
		i2c->read(rx_tx_buf, 14);

		// get raw gyro data
		acc_x = rx_tx_buf[0]<<8|rx_tx_buf[1];
		acc_y = rx_tx_buf[2]<<8|rx_tx_buf[3];
		acc_z = rx_tx_buf[4]<<8|rx_tx_buf[5];
		temp = rx_tx_buf[6]<<8|rx_tx_buf[7];
		gyro_x = rx_tx_buf[8]<<8|rx_tx_buf[9];
		gyro_y = rx_tx_buf[10]<<8|rx_tx_buf[11];
		gyro_z = rx_tx_buf[12]<<8|rx_tx_buf[13];

		// adjust for device orientation; y and z axis were flipped.
		gyro_y *= -1;
		gyro_z *= -1;

		// adjust to calibration
		if(isCallibrated) {
			gyro_x_DPS = gyro_x - gyro_sensorOffset[0];
			gyro_y_DPS = gyro_y - gyro_sensorOffset[1];
			gyro_z_DPS = gyro_z - gyro_sensorOffset[2];

			// calculate appropriate units
			acc_x_G = (double)(acc_x / acc_lsbPerG);
			acc_y_G = (double)(acc_y / acc_lsbPerG);
			acc_z_G = (double)(acc_z / acc_lsbPerG);
			gyro_x_DPS = (double)(gyro_x_DPS / gyro_lsbPerDps);
			gyro_y_DPS = (double)(gyro_y_DPS / gyro_lsbPerDps);
			gyro_z_DPS = (double)(gyro_z_DPS / gyro_lsbPerDps);
		}
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
