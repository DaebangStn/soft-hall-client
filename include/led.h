#define LED_GPIO_PIN 2

typedef enum {
    LED_OFF,
    LED_DIM,
    LED_BLINK,
    LED_ON
} LedMode_t;

extern QueueHandle_t mode_q;

void led_task(void *pvParameter);
int led_cmd(int argc, char **argv);


void _init_led(void);
uint8_t _get_duty(uint8_t count, LedMode_t mode);
const char* _get_led_mode_name(LedMode_t mode);