#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "driver/uart.h"

#include "led.h"


#define PROMPT_STR CONFIG_IDF_TARGET


esp_console_cmd_t _led_cmd = {
    .command = "led",
    .help = "LED control",
    .hint = NULL,
    .func = &led_cmd,
    .argtable = NULL
};


esp_err_t init_console(esp_console_repl_t **repl) {
    *repl = malloc(sizeof(esp_console_repl_t));
    if (*repl == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT_STR ">";
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_console_register_help_command());
    ESP_ERROR_CHECK(esp_console_cmd_register(&_led_cmd));

    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, repl));
    return ESP_OK;
}
