#include <stdio.h>
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "led.h"


static const char *TAG = "led";
QueueHandle_t mode_q = NULL;

void led_task(void *pvParameter)
{
    mode_q = *((QueueHandle_t*)pvParameter);
    LedMode_t mode = LED_OFF;
    uint8_t count = 0;

    _init_led();
    while (true) {
        TickType_t do_not_wait = 0;
        if (mode_q != NULL) {            
            ESP_LOGE(TAG, "Current led mode: %s", _get_led_mode_name(mode));
            if (xQueueReceive(mode_q, &mode, do_not_wait) == pdTRUE) {
                ESP_LOGI(TAG, "Setting led mode to %s", _get_led_mode_name(mode));
            }        
        } else {
            ESP_LOGE(TAG, "mode_q is NULL");
        }
        
        uint8_t duty = _get_duty(count, mode);
        count = (count + 1) % 100;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty)); // TODO too dim
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int led_cmd(int argc, char **argv)
{
    struct arg_int* arg_mode = arg_int0("m", "mode", "<n>", "LED mode");
    struct arg_end* end = arg_end(1);
    void* argtable[] = {arg_mode, end};

    int nerrors = arg_parse(argc, argv, argtable);
    if (nerrors != 0) {
        arg_print_errors(stderr, end, argv[0]);
        return 1;
    }

    int mode = arg_mode->ival[0];
    if (mode < LED_OFF || mode > LED_ON) {
        ESP_LOGE(TAG, "Invalid mode: %d, mode is between %d and %d", mode, LED_OFF, LED_ON);
        return 1;
    }
    if (xQueueSend(mode_q, &mode, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send mode to queue");
        return 1;
    }
    return 0;
}

void _init_led(void) {
    ledc_timer_config_t lec_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&lec_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_GPIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

uint8_t _get_duty(uint8_t count, LedMode_t mode) {
    switch (mode) {
        case LED_OFF:
            return 0;
        case LED_DIM:
            return (count % 10) * 28;
        case LED_BLINK:
            return (count % 2) * 255;
        case LED_ON:
            return 255;
        default:
            return 0;
    }
}

const char* _get_led_mode_name(LedMode_t mode) {
    switch (mode) {
        case LED_OFF:
            return "off";
        case LED_DIM:
            return "dim";
        case LED_BLINK:
            return "blink";
        case LED_ON:
            return "on";
        default:
            return "unknown";
    }
}