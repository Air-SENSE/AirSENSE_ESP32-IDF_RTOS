/**
 * @file bme280.h
 * @author Nguyen Nhu Hai Long ( @long27032002 )
 * @brief 
 * @version 0.1
 * @date 2023-01-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __BME280_H__
#define __BME280_H__

#include "esp_log.h"
#include "esp_err.h"
#include "i2cdev.h"
#include <string.h>
#include "bmp280.h"

#define TEMPERATURE_VALUE_INVALID   (float)(-273.0)
#define PRESSURE_VALUE_INVALID      (float)(-1.0)
#define HUMIDITY_VALUUE_INVALID     (float)(-1.0)

#define ID_BME280 0x02

#define ESP_ERROR_BME_INIT_FAILED       ((ID_BME280 << 12)|(0x00))
#define ESP_ERROR_BME_READ_DATA_FAILED  ((ID_BME280 << 12)|(0x01))

#define BME280_ADDRESS  BMP280_I2C_ADDRESS_0
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/

typedef  bmp280_t bme280;

esp_err_t bmp280_initialize(bmp280_t *dev, bmp280_params_t *params);

/**
 * @brief  Initialize Bme280 sensor
 * 
 * @param[in] bme280_device Device descriptor
 * @param[in] bme280_params Parameters
 * @param[in] address Address bme280 sensor
 * @param[in] i2c_port I2C port
 * @param[in] sda_gpio Pin number SDA
 * @param[in] scl_gpio Pin number SCL
 *  
 * @return esp_err_t 
 */
esp_err_t bme280_init(bme280 *bme280_device, bmp280_params_t *bme280_params,
                      uint8_t address, i2c_port_t i2c_port,
                      gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief  Read temperature, pressure, humidity from Bme280 sensor
 * 
 * @param[in] bme280_device Device descriptor
 * @param[out] temperature Temperature value
 * @param[out] pressure Pressure value
 * @param[out] humidity Humidity value
 * 
 * @return esp_err_t 
 */
esp_err_t bme280_readSensorData(bme280 *bme280_device, float *temperature,
                                float *pressure, float *humidity);

#endif