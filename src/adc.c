#include <stdio.h>
#include <unistd.h>
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
#include "bt.h"

static const char *TAG = "adc";

/*
    15                  14   13   12 
    Single shot mode    MUX2 MUX1 MUX0
    11 10 9    8 
    FSR_4096   Continous conversion mode
    7 6 5      4 
    3300SPS    ADC mode (no TS)
    3          2 1      0
    PULL_UP    VALID    Reserved

    All bits are fixed except MUX1, MUX0 (AIN selection)
*/
static const unsigned char default_tx_data[4] = {0x42 , 0xcb};
static const unsigned char mux_ain0 = 0x42;
static const unsigned char mux_ain1 = 0x52;
static const unsigned char mux_ain2 = 0x62;
static const unsigned char mux_ain3 = 0x72;
static const uint8_t resolution = 15;

static unsigned char rx_data[4];
volatile BaseType_t sem_block_drdy = pdFALSE;

struct ADC_PINS
{
    uint8_t cs[TOTAL_ADCS];
    uint8_t drdy[TOTAL_ADCS];
};

struct ADC_PINS adc_pins = {
    .cs = {PIN_NUM_CS1, PIN_NUM_CS2, PIN_NUM_CS3, PIN_NUM_CS4, PIN_NUM_CS5, PIN_NUM_CS6},
    .drdy = {PIN_NUM_DRDY1, PIN_NUM_DRDY2, PIN_NUM_DRDY3, PIN_NUM_DRDY4, PIN_NUM_DRDY5, PIN_NUM_DRDY6},
};

void init_spi(void);
spi_device_handle_t init_device(void);
void read_adc(uint8_t idx_adc, spi_device_handle_t spi, uint8_t resolution, float* voltages);
spi_transaction_t get_transaction(uint8_t num_ain);
float convert_voltage(spi_transaction_t t);
esp_err_t check_echoed_tx(spi_transaction_t t);
void pre_spi_read(uint8_t idx_adc, spi_transaction_t* tx);
void post_spi_read(uint8_t idx_adc, spi_transaction_t* tx, float* v);

void adc_task(void *pvParameter) {
    init_spi();
    spi_device_handle_t spi = init_device();
    float voltages[4] = {};
    char bt_data[SPP_DATA_LEN] = {};
    uint8_t buf_size = SPP_DATA_LEN;

    while (true) {
        uint8_t remain_size = SPP_DATA_LEN;
        for(int adc_idx=CHECKED_PIN; adc_idx<(NUM_ADCS+CHECKED_PIN); adc_idx++) {
            read_adc(adc_idx, spi, resolution, voltages);
            ESP_LOGI(TAG, "idx:%d, voltages: 0/%f, 1/%f, 2/%f, 3/%f", adc_idx, voltages[0], voltages[1], voltages[2], voltages[3]);
            if (bt_fd == -1) {
                ESP_LOGW(TAG, "bt is not connected");
                vTaskDelay(300 / portTICK_PERIOD_MS);
            } else {
                ESP_ERROR_CHECK(write_time_data(bt_data + (buf_size - remain_size), &remain_size));
                ESP_ERROR_CHECK(write_adc_data(bt_data + (buf_size - remain_size), adc_idx, voltages, &remain_size));
                ESP_LOGD(TAG, "remaining write buffer size = %d", remain_size);
                int size = write(bt_fd, bt_data, SPP_DATA_LEN);
                if (size == -1) {
                    ESP_LOGE(TAG, "write failed");
                    break;
                } else if (size == 0) {
                    ESP_LOGW(TAG, "write failed due to ringbuf is full, dropping data");
                } else {
                    ESP_LOGI(TAG, "fd = %d  data_len = %d", bt_fd, size);
                }
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
    
    for (int i=0; i<TOTAL_ADCS; i++) {
        gpio_set_level(adc_pins.cs[i], 1);
        gpio_config_t cs_cfg = {
            .pin_bit_mask = BIT64(adc_pins.cs[i]),
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&cs_cfg);
        gpio_config_t drdy_cfg = {
            .pin_bit_mask = BIT64(adc_pins.drdy[i]),
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .mode = GPIO_MODE_INPUT,
        };
        gpio_config(&drdy_cfg);
    }
    return spi;
}

void pre_spi_read(uint8_t idx_adc, spi_transaction_t* tx) {
    for(int i=0; i<6; i++) {
        tx[i] = get_transaction(i);
    }
    gpio_set_level(adc_pins.cs[idx_adc], 0);
}

void post_spi_read(uint8_t idx_adc, spi_transaction_t* tx, float* v) {
    gpio_set_level(adc_pins.cs[idx_adc], 1);
    for(int i=2; i<6; i++) {
        if(check_echoed_tx(tx[i]) != ESP_OK) {
            ESP_LOGE(TAG, "returned register values do not match sent values");
            ESP_LOGW(TAG, "sent:");
            esp_log_buffer_hex(TAG, tx[i].tx_data, 4);
            ESP_LOGW(TAG, "returned:");
            esp_log_buffer_hex(TAG, tx[i].rx_data, 4);
        }else{
            v[i-2] = convert_voltage(tx[i]);
        }
    }
}

void read_adc(uint8_t idx_adc, spi_device_handle_t spi, uint8_t resolution, float* voltages) {
    spi_transaction_t tx[6];
    pre_spi_read(idx_adc, tx);
    for (uint8_t i=0; i<6; i++) {
        int64_t now = esp_timer_get_time();
        while (gpio_get_level(adc_pins.drdy[idx_adc]) == 1) {
            if (esp_timer_get_time() - now > ADC_DRDY_TIMEOUT_US) {
                ESP_LOGE(TAG, "timeout waiting for drdy");
                break;
            }
        }
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &tx[i]));
    }
    post_spi_read(idx_adc, tx, voltages);
}

spi_transaction_t get_transaction(uint8_t num_ain) {
    unsigned char ain;
    num_ain %= 4;
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
    uint16_t data = (out[0] << 8) | out[1];
    float voltage = (float)data / (1 << resolution) * ADC_FS;
    return voltage;
}

// This function always fails when adc is in SS mode
esp_err_t check_echoed_tx(spi_transaction_t t) {
    uint8_t* out = (uint8_t*)t.rx_data;
    uint8_t* in = (uint8_t*)t.tx_data;
    if (out[2] != in[0] || out[3] != in[1]) {
        ESP_LOGE(TAG, "config echoed incorrectly");
        ESP_LOGE(TAG, "config sent:");
        esp_log_buffer_hex(TAG, in, 4);
        ESP_LOGE(TAG, "adc returned:");
        esp_log_buffer_hex(TAG, out, 4);
    }
    return ESP_OK;
}

// data must be float[4]
esp_err_t write_adc_data(char* buf, uint8_t idx_adc, float* data, uint8_t* remain_size) {
    uint8_t buf_size = *remain_size;
    for (int i=0; i<4; i++) {
        int res = snprintf(buf, buf_size, "%d-%d:%4.2f/", idx_adc, i, data[i]);
        if (res >= *remain_size || res < 0) {
            ESP_LOGE(TAG, "snprintf failed buf_size: %d res: %d", *remain_size, res);
            return ESP_FAIL;
        }
        *remain_size -= res;
        buf += res;
    }
    return ESP_OK;
}