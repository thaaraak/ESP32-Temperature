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

#include "Adafruit_ADS1X15.h"
#include "Adafruit_MAX31865.h"

#define I2C_MASTER_NUM	1
#define I2C_MASTER_SDA_IO 26
#define I2C_MASTER_SCL_IO 27

extern "C" void app_main();

#define PIN_NUM_CLK  	19
#define PIN_NUM_MISO 	35
#define PIN_NUM_MOSI 	32
#define PIN_NUM_CS   	( (gpio_num_t) 14 )
#define PIN_NUM_DIO		12
#define RESET_PIN  		13

spi_device_handle_t 	_spi;

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


void initializeSPI( int mosi, int miso, int clk, int cs )
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
	memset( &buscfg, 0, sizeof(spi_bus_config_t) );

    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = miso;
	buscfg.sclk_io_num = clk;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
	buscfg.max_transfer_sz = 0;
	buscfg.flags = 0;
	buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
	buscfg.intr_flags = 0;


    // Started working after reduce clock speed to 8MHz but then when I changed
    // back to 10 Mhz it continued working. Not sure whats going on

    spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof(spi_device_interface_config_t) );

   	devcfg.address_bits = 8;
    devcfg.mode= 1;
	devcfg.clock_speed_hz = 10000;

	//devcfg.spics_io_num=cs;
	devcfg.spics_io_num=-1;

	devcfg.flags = SPI_DEVICE_HALFDUPLEX;
	devcfg.queue_size = 1;


    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    if ( ret > 0 )
    	return;

    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);


}


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void app_main(void)
{


	i2c_master_init();
	initializeSPI( PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS );


    gpio_pad_select_gpio(PIN_NUM_CS);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);

	gpio_set_level(PIN_NUM_CS, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
	gpio_set_level(PIN_NUM_CS, 1);


	Adafruit_ADS1115 adc;
	adc.begin( I2C_MASTER_NUM );
	adc.setGain(GAIN_ONE);


	Adafruit_MAX31865 tempSensor ( _spi );
	tempSensor.begin(MAX31865_3WIRE);


    while(1)
    {

    	uint16_t adcval = adc.readADC_SingleEnded(0);
    	float volts = adc.computeVolts(adcval);


    	uint16_t adcval1 = adc.readADC_SingleEnded(1);
    	float tempvolts = adc.computeVolts(adcval1);
    	float tempTMP36 = 25 + ( tempvolts - .750 ) / .01;

    	//printf( "ADC: %d %7.5f %7.5f\n", adcval1, temp, tempvolts );



    	uint16_t rtd = tempSensor.readRTD();
    	//uint16_t rtd1 = tempSensor.readRTD();

    	float tempMAX31865 = tempSensor.temperature(RNOMINAL, RREF);
    	float ratio = rtd / 32768.0;
//    	ratio /= 32768;
    	float test = RREF*ratio;

    	printf( "Sensor: %d Temp: %12.4f Ratio: %12.4f\n", rtd, tempMAX31865, rtd / 32768.0 * RREF );
    	printf( "Temperature: (TMP36) %7.4f (MAX31865) %7.4f\n", tempTMP36, tempMAX31865 );

    	vTaskDelay(2000 / portTICK_PERIOD_MS);

    }

}

