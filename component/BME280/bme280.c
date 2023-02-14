#include "bme280.h"

__attribute__((unused)) static const char *TAG = " bme280";

static unsigned long max_timeout = 1000;

esp_err_t bme280_init(bme280 *bme280_device, bmp280_params_t *bme280_params,
                      uint8_t address, i2c_port_t i2c_port,
                      gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    memset(bme280_device, 0, sizeof(bme280));
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmp280_init_default_params(bme280_params));
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmp280_init_desc(bme280_device, address, i2c_port, sda_gpio, scl_gpio));
    ESP_ERROR_CHECK_WITHOUT_ABORT(bmp280_init(bme280_device, bme280_params));

    esp_err_t error_t;
    i2c_cmd_handle_t i2c_command = i2c_cmd_link_create();
    i2c_master_start(i2c_command);
    i2c_master_write_byte(i2c_command, address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(i2c_command);
    error_t = i2c_master_cmd_begin(i2c_port, i2c_command, (max_timeout / portTICK_RATE_MS));
    i2c_cmd_link_delete(i2c_command);

    if (error_t != ESP_OK)
    {
        ESP_LOGW(__func__, "%s: No ack, sensor not connected...skip...", esp_err_to_name(error_t));
        ESP_LOGE(__func__, "I2C port(Wire%d) for BME280 sensor initialize failed.", i2c_port);
        return ESP_ERROR_BME_INIT_FAILED;
    }

    ESP_LOGI(__func__, "BME280 sensor initialze successful.");
    return error_t;
}

esp_err_t bme280_readSensorData(bme280 *bme280_device, float *temperature,
                                float *pressure, float *humidity)
{
    bmp280_read_float(bme280_device, temperature, pressure, humidity);

    if ((*temperature < -40 || *temperature > 85)   ||
        (*pressure < 30000  || *pressure > 110000)  ||
        (*humidity < 0      || *humidity > 100  ) )
    {
        *temperature = TEMPERATURE_VALUE_INVALID;
        *pressure    = PRESSURE_VALUE_INVALID;
        *humidity    = HUMIDITY_VALUUE_INVALID;

        ESP_LOGE(__func__, "BME280 sensor read data failed.");
        return ESP_ERROR_BME_READ_DATA_FAILED;
    } else {
        ESP_LOGI(__func__, "BME280 sensor read data successful.");
        return ESP_OK;
    }
}
