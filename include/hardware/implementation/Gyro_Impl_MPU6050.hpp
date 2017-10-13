#ifndef GYRO_IMPL_6050_H_
#define GYRO_IMPL_6050_H_

#include <mraa.hpp>

#include "./../interface/Gyro_Interface.hpp"
#include "./../I2C_Slave_Device.hpp"

#define MPU6050_TX_RX_BUFFER_LENGTH 14
#define MPU6050_I2C_ADDRESS 24

class Gyro_Impl_MPU6050 : public Gyro_Interface {
private:

	int acc_x;
	int acc_y;
	int acc_z;

	int gyro_x;
	int gyro_y;
	int gyro_z;

	int temp;

public:

	Gyro_Impl_MPU6050(int i2c_address, mraa::I2c* i2c_controller) : Gyro_Interface(i2c_address, i2c_controller) {
		std::cout << "MPU6050 - Constructor" << std::endl;
		acc_x = 0;
		acc_y = 0;
		acc_z = 0;
		gyro_x = 0;
		gyro_y = 0;
		gyro_z = 0;
		temp = 0;
	}

	void read() {
		i2c->address(i2c_address); // set talking slave (before each read/write)
		rx_tx_buf[0] = 0x3B;	// Read from Register 0x1B
		i2c->write(rx_tx_buf, 1);

		i2c->address(i2c_address); // set talking slave (before each read/write)
		i2c->read(rx_tx_buf, 14);

		acc_x = rx_tx_buf[0]<<8|rx_tx_buf[1];
		acc_y = rx_tx_buf[2]<<8|rx_tx_buf[3];
		acc_z = rx_tx_buf[4]<<8|rx_tx_buf[5];
		temp = rx_tx_buf[6]<<8|rx_tx_buf[7];
		gyro_x = rx_tx_buf[8]<<8|rx_tx_buf[9];
		gyro_y = rx_tx_buf[10]<<8|rx_tx_buf[11];
		gyro_z = rx_tx_buf[12]<<8|rx_tx_buf[13];
		gyro_x *= -1;
	}

	void init() {

		bool didSucceed = false;
		while(!didSucceed) {
			std::cout << "MPU6050 - INIT" << std::endl;

			/*
				i2c->address(i2c_address); // set talking slave (before each read/write)
				rx_tx_buf[0] = 0x6B;	// Write to PWR_MGMT_1 Register
				rx_tx_buf[1] = 0x00;	// Activate gyro by writing 0 to register bits
				i2c->write(rx_tx_buf, 2);


				i2c->address(i2c_address); // set talking slave (before each read/write)
				rx_tx_buf[0] = 0x1B;	// Write to GYRO_CONFIG Register
				rx_tx_buf[1] = 0x08;	// Set Register for 500dps @ full scale
				i2c->write(rx_tx_buf, 2);

				i2c->address(i2c_address); // set talking slave (before each read/write)
				rx_tx_buf[0] = 0x1B;	// Write to ACCEL_CONFIG Register
				rx_tx_buf[1] = 0x08;	// Set Register for +/- 8g @ full scale
				i2c->write(rx_tx_buf, 2);

				// SELF-CHECK: read back configurations
				i2c->address(i2c_address); // set talking slave (before each read/write)
				rx_tx_buf[0] = 0x1B;	// Read from Register 0x1B
				i2c->write(rx_tx_buf, 1);

				i2c->address(i2c_address); // set talking slave (before each read/write)
				i2c->read(rx_tx_buf, 1);

				if(rx_tx_buf[0] != 0x08) {
					std::cout << "Failed to initialize GYRO_MPU6050" << std::endl;
				} else {
					didSucceed = true;
				}
			*/

			// Write to PWR_MGMT_1 Register - 0x6B
			// Activate gyro by writing 0 to register bits
			writeByteToAddress(0x6B, 0x00);


			// Write to GYRO_CONFIG Register - 0x1B
			// Set Register for 500dps @ full scale
			writeByteToAddress(0x1B, 0x08);

			// Write to ACCEL_CONFIG Register - 0x1C
			// Set Register for +/- 8g @ full scale
			writeByteToAddress(0x1C, 0x10);

			// Write to CONFIG Register - 0x1A
			// Set Register for Digital Low Pass Filter
			writeByteToAddress(0x1A, 0x03);

			// SELF-CHECK: read back configurations
			// Read from Register 0x1B
			if(readByteValueFromAddress(0x1A) != 0x03) {
				std::cout << "Failed to initialize GYRO_MPU6050" << std::endl;
				//usleep(500);
			} else {
				didSucceed = true;
			}
		}
	}

	int16_t getAcc_x() {
		return acc_x;
	}
	int16_t getAcc_y() {
		return acc_y;
	}
	int16_t getAcc_z() {
		return acc_z;
	}

	int16_t getGyro_roll() {
		return gyro_x;
	}
	int16_t getGyro_pitch() {
		return gyro_y;
	}
	int16_t getGyro_yaw() {
		return gyro_z;
	}

	int16_t getTemp() {
		return temp;
	}

	float getOutputForOneDegreePerSecond() {
		return 65.5;
	}

	~Gyro_Impl_MPU6050() {
		std::cout << "MPU6050 - DESTROY" << std::endl;
	}

};

#endif // GYRO_IMPL_6050_H_
