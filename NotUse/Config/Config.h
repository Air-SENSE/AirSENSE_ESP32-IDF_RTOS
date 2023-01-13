/**
 * @file Config.h
 * @author Nguyen Nhu Hai Long
 * @brief Define PIN, PORT
 * @version 0.1
 * @date 2022-11-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define DEVICE_NAME "AirSENSE_RTOS_ESP-IDF"
#define FIRMWARE_VERSION "0.0.0"

#define USING_BME280
#define USING_DS3231
#define PIN_NUM_SDA 26
#define PIN_NUM_SCL 27

#define USING_SD_CARD
#define PIN_NUM_MISO    21
#define PIN_NUM_MOSI    19
#define PIN_NUM_CLK     18
#define PIN_NUM_CS      15


#define USING_MQTT
#define PORT 1883U

/**}*/

#endif