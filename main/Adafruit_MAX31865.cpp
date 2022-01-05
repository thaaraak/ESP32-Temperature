/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31865.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"

static const char *TAG = "MAX31865";

#define PIN_NUM_CS   	( (gpio_num_t) 14 )



/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
*/
/**************************************************************************/
//
Adafruit_MAX31865::Adafruit_MAX31865( spi_device_handle_t spi )
{
	_spi = spi;
}


/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool Adafruit_MAX31865::begin(max31865_numwires_t wires) {

  vTaskDelay(30 / portTICK_PERIOD_MS);

  setWires(wires);
  //enableBias(false);
  enableBias(true);
  autoConvert(false);
  clearFault();

  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  //printf( "**** Config ****  [%d]\n", t );

  return true;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::readFault(void) {
  return readRegister8(MAX31865_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865::clearFault(void) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31865_CONFIG_FAULTSTAT;
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void Adafruit_MAX31865::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/

void Adafruit_MAX31865::enable50Hz(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_FILT50HZ;
  } else {
    t &= ~MAX31865_CONFIG_FILT50HZ;
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void Adafruit_MAX31865::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);

  if (wires == MAX31865_3WIRE) {
    t |= MAX31865_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31865_CONFIG_3WIRE;
  }

  //printf( "Setting Wires to: %d\n", t );
  writeRegister8(MAX31865_CONFIG_REG, t);
  t = readRegister8(MAX31865_CONFIG_REG);

}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float Adafruit_MAX31865::temperature(float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD(void)

{
	//printf( "Clearing Fault\n");
	clearFault();

	//printf( "Enable Bias True\n");
	//enableBias(true);
	//vTaskDelay(100 / portTICK_PERIOD_MS);

	//printf( "Config 1 Shot\n");
	uint8_t t = readRegister8(MAX31865_CONFIG_REG);
	t |= MAX31865_CONFIG_1SHOT;
	writeRegister8(MAX31865_CONFIG_REG, t);
	//vTaskDelay(100 / portTICK_PERIOD_MS);

	//printf( "Reading RTDMSB\n");
	uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);
	//uint16_t rtdlow = readRegister(MAX31865_RTDLSB_REG);

	//enableBias(false); // Disable bias current again to reduce selfheating.

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/

uint8_t Adafruit_MAX31865::readRegister8(uint8_t addr) {
  return readRegister(addr );
}

uint16_t Adafruit_MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  writeRegister( addr, data );
}


void Adafruit_MAX31865::writeRegister( uint8_t reg, uint8_t data )
{
	gpio_set_level(PIN_NUM_CS, 0);

	reg = reg | 0x80;
    //printf("Writing Register [%02x]=[%02x]\n", reg, data);

	spi_transaction_t transaction;
	memset( &transaction, 0, sizeof(spi_transaction_t) );

	transaction.length = 8;
	transaction.rxlength = 0;
	transaction.addr = reg;
	transaction.flags = SPI_TRANS_USE_TXDATA;

	memcpy(transaction.tx_data, &data, 1);

	esp_err_t err = spi_device_polling_transmit(_spi, &transaction);

	if (err != ESP_OK)
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));

	gpio_set_level(PIN_NUM_CS, 1);

}

uint8_t Adafruit_MAX31865::readRegister( uint8_t reg )
{
	uint8_t result;

	gpio_set_level(PIN_NUM_CS, 0);

	spi_transaction_t transaction;
	memset( &transaction, 0, sizeof(spi_transaction_t) );

	transaction.length = 0;
	transaction.rxlength = 8;
	transaction.addr = reg & 0x7f;
	transaction.flags = SPI_TRANS_USE_RXDATA;

	esp_err_t err = spi_device_polling_transmit( _spi, &transaction);

	if (err != ESP_OK)
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));

	memcpy(&result, transaction.rx_data, 1);
    //printf("Reading Register rxlen: %d [%02x]=[%02x]\n", transaction.rxlength, reg, result);

	gpio_set_level(PIN_NUM_CS, 1);

	return result;
}

void Adafruit_MAX31865::readRegisterN(uint8_t reg, uint8_t buffer[], uint8_t len) {

	uint8_t result;

	gpio_set_level(PIN_NUM_CS, 0);

	spi_transaction_t transaction;
	memset( &transaction, 0, sizeof(spi_transaction_t) );

	transaction.length = 0;
	transaction.rxlength = len * 8;
	transaction.addr = reg & 0x7f;
	transaction.flags = SPI_TRANS_USE_RXDATA;

	esp_err_t err = spi_device_polling_transmit( _spi, &transaction);

	if (err != ESP_OK)
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));

	gpio_set_level(PIN_NUM_CS, 1);

	memcpy(buffer, transaction.rx_data, len);


	if ( len > 1 ) {
		printf("Reading Register len:(%d) rx:(%d) [%02x]=[", len, transaction.rxlength, reg );

		for ( int i = 0 ; i < len ; i++ )
			printf("%02x:", transaction.rx_data[i]);

		printf( "]\n");
	}



}


