#ifndef I2C_SLAVE_DEVICE_HPP_
#define I2C_SLAVE_DEVICE_HPP_

#include <unistd.h>
#include <signal.h>

class I2C_Slave_Device {

protected:
	uint8_t i2c_address;
	mraa::I2c* i2c;
	uint8_t rx_tx_buf[50];

	void writeValueToAddress(uint8_t address, uint8_t* rxTxBuffer, int txBufferSize) {
		uint8_t* txBuffer = new uint8_t[txBufferSize + 1];
		for(int i = 0; i < txBufferSize; i++) {
			txBuffer[i+1] = rxTxBuffer[i];
		}
		txBuffer[0] = address;
		i2c->address(i2c_address); // set talking slave (before each read/write)
		i2c->write(txBuffer, txBufferSize+1);
		delete txBuffer;
	}

	void writeByteToAddress(uint8_t address, uint8_t txValue) {
		rx_tx_buf[0] = address;
		rx_tx_buf[1] = txValue;
		i2c->address(i2c_address); // set talking slave (before each read/write)
		i2c->write(rx_tx_buf, 2);
	}

	void writeBuffer(uint8_t* rxTxBuffer, int txBufferSize) {
		i2c->address(i2c_address); // set talking slave (before each read/write)
		i2c->write(rxTxBuffer, txBufferSize);
	}

	uint8_t readByteValueFromAddress(uint8_t address) {
		i2c->address(i2c_address);
		rx_tx_buf[0] = address;	// Read from specified register
		i2c->write(rx_tx_buf, 1);

		i2c->address(i2c_address);
		i2c->read(rx_tx_buf, 1);
		return rx_tx_buf[0];
	}

	void readValueFromAddress(uint8_t address, uint8_t* rxTxBuffer, int rxBufferSize) {
		i2c->address(i2c_address);
		rxTxBuffer[0] = address;	// Read from specified register
		i2c->write(rxTxBuffer, 1);

		i2c->address(i2c_address);
		i2c->read(rxTxBuffer, rxBufferSize);
	}

public:

	class METHOD_NOT_IMPLEMENTED: public std::exception
	{
		virtual const char* getException() const throw() {
			return "Method not implemented";
		}
	} methodNotImplementedException;

	virtual void init(){
		throw methodNotImplementedException;
	}

	virtual ~I2C_Slave_Device() {
	}

	I2C_Slave_Device() {
		i2c_address = 0;
		i2c = new mraa::I2c(0);
	}

	I2C_Slave_Device(int address, mraa::I2c* i2c_controller){
		i2c_address = address;
		i2c = i2c_controller;
	}

	void setAddress(int address) {
		i2c_address = address;
	}

	int getAddress() {
		return i2c_address;
	}
};

#endif // I2C_SLAVE_DEVICE_HPP_
