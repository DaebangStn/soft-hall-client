#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "adc.h"

static const char *TAG = "adc";


static void init_spi(void);
static spi_device_handle_t init_device(void);
static void read_adc(spi_device_handle_t spi, uint8_t resolution, float* voltages);
// spi_transaction_t _get_transaction(uint8_t num_ain);
static esp_err_t check_drdy(uint16_t timeout_us);
static float convert_voltage(uint8_t resolution, spi_transaction_t t);
static esp_err_t check_echoed_tx(spi_transaction_t t);


void adc_task(void *pvParameter) {
    init_spi();
    spi_device_handle_t spi = init_device();
    while (true) {
        float voltages[4] = {};
        read_adc(spi, 16, voltages);
        ESP_LOGI(TAG, "voltages: %f, %f, %f, %f", voltages[0], voltages[1], voltages[2], voltages[3]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


static void init_spi(void) {
    ESP_LOGI(TAG, "initializing bus SPI%d...", SPI_BUS);
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BUS, &buscfg, SPI_DMA_DISABLED));
}

static spi_device_handle_t init_device(void) {
    ESP_LOGI(TAG, "initializing device...");
    int ain_per_adc = 4;
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 1,
        .input_delay_ns = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = ain_per_adc,
    };
    spi_device_handle_t spi = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_config, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
    return spi;
}

static void read_adc(spi_device_handle_t spi, uint8_t resolution, float* voltages) {
    for (uint8_t i=0; i<4; i++) {
        // spi_transaction_t t = _get_transaction(i);
        ESP_ERROR_CHECK(check_drdy(ADC_DRDY_TIMEOUT_US));
        // ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
        // ESP_ERROR_CHECK(_check_echoed_tx(t));
        // voltages[i] = _convert_voltage(resolution, t);
    }
}

// spi_transaction_t _get_transaction(uint8_t num_ain) {
//     unsigned char tx_data[4] = {0};
//     unsigned char rx_data[4] = {0};
//     spi_transaction_t t = {
//         .flags = SPI_TRANS_CS_KEEP_ACTIVE | SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
//         .cmd = 0,
//         .addr = 0,
//         .length = 32,
//         .user = (void*)0,
//         .tx_data = tx_data,
//         .rx_data = rx_data,
//     };
//     return t;
// }

static esp_err_t check_drdy(uint16_t timeout_us) {
    // spin until drdy low
    return ESP_OK;
}

static float convert_voltage(uint8_t resolution, spi_transaction_t t) {
    return 0.0;
}

static esp_err_t check_echoed_tx(spi_transaction_t t) {
    return ESP_OK;
}

esp_err_t write_adc_data(char* buf, uint16_t data, uint8_t* remain_size) {
    uint8_t buf_size = *remain_size;
    uint8_t res = snprintf(buf, buf_size, "0:%"PRId16"/", data);
    if (res >= *remain_size || res < 0) {
        ESP_LOGE(TAG, "snprintf failed buf_size: %d res: %d", *remain_size, res);
        return ESP_FAIL;
    }
    *remain_size -= res;
    return ESP_OK;
}