#include <stdio.h>
#include "esp_err.h"

#define PIN_NUM_MISO      19
#define PIN_NUM_MOSI      23
#define PIN_NUM_CLK       18
#define PIN_NUM_CS        5
#define PIN_NUM_DRDY      9

#define SPI_BUS           SPI2_HOST

#define ADC_DRDY_TIMEOUT_US  100

void adc_task(void *pvParameter);
esp_err_t write_adc_data(char* buf, uint16_t data, uint8_t* remain_size);