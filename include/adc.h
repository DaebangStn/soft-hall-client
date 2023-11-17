#define PIN_NUM_MISO      13
#define PIN_NUM_MOSI      12
#define PIN_NUM_CLK       11
#define PIN_NUM_CS        10
#define PIN_NUM_DRDY      9

#define SPI_BUS           SPI2_HOST

#define ADC_DRDY_TIMEOUT_US  100


void _init_spi(void);
spi_device_handle_t _init_device(void);
void _read_adc(spi_device_handle_t spi, uint8_t resolution, float* voltages);
// spi_transaction_t _get_transaction(uint8_t num_ain);
esp_err_t _check_drdy(uint16_t timeout_us);
float _convert_voltage(uint8_t resolution, spi_transaction_t t);
esp_err_t _check_echoed_tx(spi_transaction_t t);