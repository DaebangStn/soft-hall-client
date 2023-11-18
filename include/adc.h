#include <stdio.h>
#include "esp_err.h"

#define PIN_NUM_MISO      13
#define PIN_NUM_MOSI      12
#define PIN_NUM_CLK       11
#define PIN_NUM_CS        10
#define PIN_NUM_DRDY      9

#define SPI_BUS           SPI2_HOST

#define ADC_DRDY_TIMEOUT_US  100

esp_err_t write_adc_data(char* buf, uint16_t data, uint8_t buf_size);