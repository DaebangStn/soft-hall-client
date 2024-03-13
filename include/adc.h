#include <stdio.h>
#include "esp_err.h"

#define PIN_NUM_MISO      19
#define PIN_NUM_MOSI      23
#define PIN_NUM_CLK       18

#define PIN_NUM_CS1       32
#define PIN_NUM_DRDY1     35
#define PIN_NUM_CS2       21
#define PIN_NUM_DRDY2     22
#define PIN_NUM_CS3       27
#define PIN_NUM_DRDY3     14
#define PIN_NUM_CS4       16
#define PIN_NUM_DRDY4     17
#define PIN_NUM_CS5       10
#define PIN_NUM_DRDY5     9
#define PIN_NUM_CS6       2
#define PIN_NUM_DRDY6     15

#define SPI_BAUDRATE      2000000
#define SPI_BUS           SPI2_HOST
/*
    to change this value, 
    you must also change the value of default_tx_data in adc.c
*/
#define ADC_FS            4096


#define ADC_DRDY_TIMEOUT_US  100000

#define NUM_ADCS          2
#define TOTAL_ADCS        6
#define CHECKED_PIN       0

void adc_task(void *pvParameter);
esp_err_t write_adc_data(char* buf, uint8_t idx_adc, float* data, uint8_t* remain_size);