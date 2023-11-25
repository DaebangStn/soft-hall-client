#include <stdio.h>
#include "esp_err.h"

#define PIN_NUM_MISO      19
#define PIN_NUM_MOSI      23
#define PIN_NUM_CLK       18
#define PIN_NUM_CS        5
#define PIN_NUM_DRDY      10

#define SPI_BAUDRATE      1000000
#define SPI_BUS           SPI2_HOST
/*
    to change this value, 
    you must also change the value of default_tx_data in adc.c
*/
#define ADC_FS            4096


#define ADC_DRDY_TIMEOUT_US  100000

void adc_task(void *pvParameter);
esp_err_t write_adc_data(char* buf, uint16_t data, uint8_t* remain_size);