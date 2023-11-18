#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"


esp_err_t init_nvs(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    return ESP_OK;
}