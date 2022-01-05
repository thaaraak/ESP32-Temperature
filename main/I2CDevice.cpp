#include "I2CDevice.h"
#include <stdint.h>


#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL I2C_MASTER_ACK
#define NACK_VAL I2C_MASTER_NACK

I2CDevice::I2CDevice(int i2c_num, uint8_t i2c_addr )
{
	i2c_bus_addr = i2c_addr;
	_i2c_num = i2c_num;
}

uint8_t I2CDevice::write_bulk(uint8_t reg, uint8_t *data, uint8_t bytes )
{
	/*
    printf( "Writing [%02x]=", reg );
    for ( int i = 0 ; i < bytes ; i++ )
        printf( "%02x:", data[i] );
    printf( "\n");
	*/

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, &reg, 1, ACK_CHECK_EN);
    i2c_master_write(cmd, data, bytes, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin( _i2c_num, cmd, 1000 / portTICK_RATE_MS);

    //printf( "Return: %d\n:", ret );

    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    return 0;
}

uint8_t I2CDevice::write(uint8_t reg, uint8_t value)
{
    return write_bulk( reg, &value, 1 );
}



uint8_t I2CDevice::read(uint8_t reg)
{
	   	uint8_t buffer[1];
	   	read_bulk( reg, &buffer[0], 1 );
	   	return buffer[0];
}

uint8_t I2CDevice::read_bulk(uint8_t reg, uint8_t* buffer, int len )
{
	/*
		if ( reg == 0 )
			printf( "Reading register (len=%d): [%d] = ", len, reg );
*/
	    int ret;
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	    // Write the register address to be read
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

	    // Read the data for the register from the slave
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, i2c_bus_addr << 1 | READ_BIT, ACK_CHECK_EN);

	    if (len > 1)
	        i2c_master_read(cmd, &buffer[0], len - 1, ACK_VAL);
	    i2c_master_read_byte(cmd, buffer + len - 1, NACK_VAL);

	    i2c_master_stop(cmd);

	    esp_err_t i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, TickType_t ticks_to_wait);

	    ret = i2c_master_cmd_begin( (i2c_port_t)_i2c_num, cmd, (TickType_t) 1000 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);
/*
		if ( reg == 0 )
			printf( "%d %d\n", buffer[0], buffer[1] );
*/
	    return (buffer[0]);
}
