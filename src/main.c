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
#include "adc.h"
#include "my_nvs.h"
#include "led.h"
#include "console.h"


static const char *TAG = "main";


// void write_task(void *pvParameter) {
//     int size = 0;
//     char bt_data[SPP_DATA_LEN] = {};
//     uint8_t buf_size = SPP_DATA_LEN;
//     uint16_t i = 0;

//     while (true) {
//         uint8_t remain_size = SPP_DATA_LEN;
//         ESP_ERROR_CHECK(write_time_data(bt_data + (buf_size - remain_size), &remain_size));
//         ESP_ERROR_CHECK(write_adc_data(bt_data + (buf_size - remain_size), i++, &remain_size));
//         ESP_LOGI(TAG, "remain_size = %d", remain_size);
//         if (bt_fd == -1) {
//             ESP_LOGE(TAG, "bt_fd == -1");
//             vTaskDelay(5000 / portTICK_PERIOD_MS);
//         } else {
//             size = write(bt_fd, bt_data, SPP_DATA_LEN);
//             if (size == -1) {
//                 break;
//             } else if ( size == 0) {
//                 /*write fail due to ringbuf is full, retry after 500 ms*/
//                 vTaskDelay(500 / portTICK_PERIOD_MS);
//             } else {
//                 ESP_LOGI(TAG, "fd = %d  data_len = %d", bt_fd, size);
//                 vTaskDelay(1000 / portTICK_PERIOD_MS);
//             }
//         }
//     }
// }


void app_main() {
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(bt_init());

    // due to the led pin duplicating with the spi pin, 
    // we cannot use the led task
    // ESP_LOGI(TAG, "initializing led...");
    // QueueHandle_t led_mode_q = xQueueCreate(5, sizeof(LedMode_t));
    // if(led_mode_q == 0) {
    //     ESP_LOGE(TAG, "led_mode_q xQueueCreate failed");
    //     exit(1);
    // }
    // xTaskCreate(led_task, "led", 2048, &led_mode_q, 1, NULL);

    QueueHandle_t bt_task_msg_q = xQueueCreate(5, sizeof(bt_task_msg_t));
    if(bt_task_msg_q == 0) {
        ESP_LOGE(TAG, "bt_task_msg_q xQueueCreate failed");
        exit(1);
    }
    xTaskCreate(bt_task, "bt", 4096, &bt_task_msg_q, 1, NULL);

    ESP_LOGI(TAG, "initializing adc...");
    xTaskCreate(adc_task, "adc", 4096, NULL, 1, NULL);

    // used when console is online 
    // esp_console_repl_t *repl = NULL;
    // ESP_ERROR_CHECK(init_console(&repl));
    // ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // dummy task
    // xTaskCreate(write_task, "write", 4096, NULL, 1, NULL);
}