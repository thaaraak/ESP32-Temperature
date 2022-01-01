
#ifndef I2C_DEVICE_H_
#define I2C_DEVICE_H_

#include <stdint.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"


class I2CDevice
{
public:
	I2CDevice( int i2c_num, uint8_t i2c_addr );

	uint8_t write_bulk(uint8_t reg, uint8_t* buffer, uint8_t len );
	uint8_t write(uint8_t, uint8_t);

	uint8_t read_bulk(uint8_t reg, uint8_t* buffer, int len );
	uint8_t read(uint8_t reg);

private:
	uint8_t i2c_bus_addr;
	int _i2c_num;
};

#endif
