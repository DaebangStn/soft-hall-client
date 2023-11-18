#include <stdio.h>
#include <unistd.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "driver/uart.h"

#include "esp_console.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "bt.h"
#include "my_nvs.h"
#include "led.h"
#include "console.h"


void write_task(void *pvParameter) {
    int size = 0;
    uint8_t *spp_data = NULL;
    uint16_t i = 0;

    spp_data = malloc(SPP_DATA_LEN);
    if (!spp_data) {
        ESP_LOGE("TAG", "malloc bt_data failed, fd:%d", bt_fd);
        goto done;
    }

    for (i = 0; i < SPP_DATA_LEN; ++i) {
        spp_data[i] = 'a';
    }

    while (true) {
        if (bt_fd == -1) {
            ESP_LOGE("TAG", "bt_fd == -1");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        } else {
            ESP_LOGI("TAG", "bt_fd = %d", bt_fd);
            size = write(bt_fd, spp_data, SPP_DATA_LEN);
            if (size == -1) {
                break;
            } else if ( size == 0) {
                /*write fail due to ringbuf is full, retry after 500 ms*/
                vTaskDelay(500 / portTICK_PERIOD_MS);
            } else {
                ESP_LOGI("TAG", "fd = %d  data_len = %d", bt_fd, size);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
done:
    if (spp_data) {
        free(spp_data);
    }
    vTaskDelete(NULL);
}


void app_main() {
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(bt_init());
    QueueHandle_t bt_task_msg_q = xQueueCreate(5, sizeof(bt_task_msg_t));
    xTaskCreate(bt_task, "bt", 4096, &bt_task_msg_q, 1, NULL);
    QueueHandle_t led_mode_q = xQueueCreate(1, sizeof(LedMode_t));
    xTaskCreate(led_task, "led", 2048, &led_mode_q, 1, NULL);

    esp_console_repl_t *repl = NULL;
    ESP_ERROR_CHECK(init_console(&repl));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    xTaskCreate(write_task, "write", 2048, NULL, 1, NULL);
}