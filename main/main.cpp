#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "Adafruit_ADS1X15.h"

#define I2C_MASTER_NUM	0
#define I2C_MASTER_SDA_IO 26
#define I2C_MASTER_SCL_IO 27

extern "C" void app_main();


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void app_main(void)
{
	i2c_master_init();

	Adafruit_ADS1115 adc;
	adc.begin( I2C_MASTER_NUM );
	adc.setGain(GAIN_ONE);

    while(1)
    {
    	uint16_t adcval = adc.readADC_SingleEnded(0);
    	float volts = adc.computeVolts(adcval);

    	uint16_t adcval1 = adc.readADC_SingleEnded(1);
    	float tempvolts = adc.computeVolts(adcval1);
    	float temp = 25 + ( tempvolts - .750 ) / .01;

    	printf( "ADC: %d %7.5f %7.5f\n", adcval1, temp, tempvolts );
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

