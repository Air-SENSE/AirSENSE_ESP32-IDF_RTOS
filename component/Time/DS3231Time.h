/**
 * @file DS3231Time.h
 * @author Nguyen Nhu Hai Long ( @long27032002 )
 * @brief All function for DS3231 RTC
 * @version 0.1
 * @date 2022-12-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DS3231TIME_H__
#define __DS3231TIME_H__

#include <esp_log.h>
#include <time.h>
#include <stdbool.h>
#include <esp_err.h>
#include <string.h>
#include "ds3231.h"

#define SECONDS_PER_MIN             60ULL
#define SECONDS_PER_HOUR            3600ULL
#define SECONDS_PER_DAY             86400ULL
#define SECONDS_PER_MON             2629743ULL
#define SECONDS_PER_YEAR            31556926ULL
#define SECONDS_FROM_1970_TO_2023   1672506000U    // Unixtime for 2023-01-01 00:00:00

int currentDay;

/**
 * @brief Initialize device descriptor
 * 
 * @param[in] dev I2C device descriptor
 * @param[in] port I2C port
 * @param[in] sda_gpio SDA GPIO
 * @param[in] scl_gpio SCL GPIO
 * 
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_initialize(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * 
 * @param[in] dev I2C device descriptor
 * 
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_free_desc(i2c_dev_t *dev);

/**
 * @brief Set the time on the RTC
 *
 * Timezone agnostic, pass whatever you like.
 * I suggest using GMT and applying timezone and DST when read back.
 * @param[in] dev Device descriptor
 * @param[out] time RTC time
 * 
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_setTime(i2c_dev_t *dev, struct tm *time);

/**
 * @brief Get the time from the RTC, populates a supplied tm struct
 * 
 * @param[in] dev Device descriptor
 * @param[out] time RTC time
 * 
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_get_time(i2c_dev_t *dev, struct tm *time);

/**
 * @brief Get the time from the RTC, populates a supplied tm struct
 * 
 * @param[in] dev Device descriptor
 * @param[out] timeString time string
 * @param[in] lenghtString lenght of time string
 * 
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_convertTimeToString(i2c_dev_t *dev, char* timeString, const unsigned int lenghtString);

/**
 * @brief Get the time from the RTC, populates a supplied tm struct
 * 
 * @param[in] dev Device descriptor
 * @param[out] epochTime unix time
 * 
 * @return ESP_OK to indicate success
 */
esp_err_t ds3231_getEpochTime(i2c_dev_t *dev, uint32_t *epochTime);


bool ds3231_isNewDay(i2c_dev_t *dev);

#endif