#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "adc.h"

static const char *TAG = "adc";

/*
    15                  14   13   12 
    Single shot mode    MUX2 MUX1 MUX0
    11 10 9    8 
    FSR_4096   Continous conversion mode
    7 6 5      4 
    868SPS     ADC mode (no TS)
    3          2 1      0
    PULL_UP    VALID    Reserved

    All bits are fixed except MUX1, MUX0 (AIN selection)
*/
static const unsigned char default_tx_data[4] = {0x82 , 0xeb};
static const unsigned char mux_ain0 = 0x42;
static const unsigned char mux_ain1 = 0x52;
static const unsigned char mux_ain2 = 0x62;
static const unsigned char mux_ain3 = 0x72;
static const uint8_t resolution = 15;

static spi_transaction_t tx[4] = {};
static unsigned char rx_data[4];
volatile BaseType_t sem_block_drdy = pdFALSE;

void init_spi(void);
spi_device_handle_t init_device(void);
void read_adc(spi_device_handle_t spi, uint8_t resolution, float* voltages);
spi_transaction_t get_transaction(uint8_t num_ain);
esp_err_t check_drdy(uint16_t timeout_us);
float convert_voltage(spi_transaction_t t);
esp_err_t check_echoed_tx(spi_transaction_t t);
void pre_spi_read(void);
void post_spi_read(float* voltages);

void IRAM_ATTR isr_check_drdy(void* arg) {
    sem_block_drdy = pdTRUE;
}

void adc_task(void *pvParameter) {
    float voltages[4] = {};
    ESP_LOGI(TAG, "voltages: %f, %f, %f, %f", voltages[0], voltages[1], voltages[2], voltages[3]);
    init_spi();
    spi_device_handle_t spi = init_device();
    while (true) {
        float voltages[4] = {};
        read_adc(spi, 16, voltages);
        ESP_LOGI(TAG, "voltages: %f, %f, %f, %f", voltages[0], voltages[1], voltages[2], voltages[3]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void init_spi(void) {
    ESP_LOGI(TAG, "initializing bus SPI%d...", SPI_BUS);
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BUS, &buscfg, SPI_DMA_CH_AUTO));
}

spi_device_handle_t init_device(void) {
    ESP_LOGI(TAG, "initializing device...");
    int ain_per_adc = 4;
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = SPI_BAUDRATE,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = ain_per_adc,
        .input_delay_ns = 50,
    };
    spi_device_handle_t spi = NULL;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_config, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
    
    gpio_set_level(PIN_NUM_CS, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);
    gpio_config_t drdy_cfg = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = BIT64(PIN_NUM_DRDY),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&drdy_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DRDY, isr_check_drdy, NULL);
    gpio_intr_disable(PIN_NUM_DRDY);

    return spi;
}

void pre_spi_read(void) {
    gpio_set_level(PIN_NUM_CS, 0);
    gpio_intr_enable(PIN_NUM_DRDY);
    sem_block_drdy = pdFALSE;
    for (int i=0; i<4; i++) {
        tx[i] = get_transaction(i + 1);
    }
}

void post_spi_read(float* voltages) {
    gpio_set_level(PIN_NUM_CS, 1);
    gpio_intr_disable(PIN_NUM_DRDY);

    for (int i=0; i<4; i++) {
        if (check_echoed_tx(tx[i]) == ESP_OK) {
            voltages[i] = convert_voltage(tx[i]);
        }
    }
}

void read_adc(spi_device_handle_t spi, uint8_t resolution, float* voltages) {
    pre_spi_read();
    for (uint8_t i=0; i<4; i++) {
        int64_t now = esp_timer_get_time();
        while (sem_block_drdy == pdFALSE) {
            if (esp_timer_get_time() - now > ADC_DRDY_TIMEOUT_US) {
                ESP_LOGE(TAG, "timeout waiting for drdy");
                break;
            }
        }
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &tx[i]));
        sem_block_drdy = pdFALSE; // semaphore is set while transaction is in progress
    }
    post_spi_read(voltages);
}

spi_transaction_t get_transaction(uint8_t num_ain) {
    unsigned char ain;
    switch (num_ain) {
        case 0:
            ain = mux_ain0;
            break;
        case 1:
            ain = mux_ain1;
            break;
        case 2:
            ain = mux_ain2;
            break;
        case 3:
            ain = mux_ain3;
            break;
        case 4:
            ain = mux_ain0;
            break;
        default:
            ESP_LOGE(TAG, "invalid num_ain: %d", num_ain);
            ain = mux_ain0;
    }
    spi_transaction_t t = {
        .length = 32,
        .rxlength = 32,
        .user = (void*)0,
        .tx_data = {ain, default_tx_data[1], ain, default_tx_data[1]},
        .rx_buffer = rx_data,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
    };
    return t;
}

float convert_voltage(spi_transaction_t t) {
    uint8_t* out = (uint8_t*)t.rx_data;
    if (out[0] == 0x00 && out[1] == 0x00) {
        ESP_LOGW(TAG, "adc conversion failed");
        ESP_LOGI(TAG, "adc returned:");
        esp_log_buffer_hex(TAG, out, 4);
        return 0.0;
    }
    uint16_t data = (out[0] << 8) | out[1];
    float voltage = (float)data / (1 << resolution) * ADC_FS;
    return voltage;
}

// This function always fails when adc is in SS mode
esp_err_t check_echoed_tx(spi_transaction_t t) {
    ESP_LOGI(TAG, "checking config echoed...");
    uint8_t* out = (uint8_t*)t.rx_data;
    uint8_t* in = (uint8_t*)t.tx_data;
    if (out[2] != in[0] || out[3] != in[1]) {
        ESP_LOGE(TAG, "config echoed incorrectly");
        ESP_LOGE(TAG, "config sent:");
        esp_log_buffer_hex(TAG, in, 4);
        ESP_LOGE(TAG, "adc returned:");
        esp_log_buffer_hex(TAG, out, 4);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t write_adc_data(char* buf, uint16_t data, uint8_t* remain_size) {
    uint8_t buf_size = *remain_size;
    int res = snprintf(buf, buf_size, "0:%"PRId16"/", data);
    if (res >= *remain_size || res < 0) {
        ESP_LOGE(TAG, "snprintf failed buf_size: %d res: %d", *remain_size, res);
        return ESP_FAIL;
    }
    *remain_size -= res;
    return ESP_OK;
}