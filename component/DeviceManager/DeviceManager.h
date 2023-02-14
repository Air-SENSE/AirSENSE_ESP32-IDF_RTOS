/**
 * @file DeviceManager.h
 * @author Nguyen Nhu Hai Long 
 * @brief Manager all device 
 * @version 0.1
 * @date 2022-11-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __DEVICEMANAGER_H__
#define __DEVICEMANAGER_H__

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "string.h"
#include "time.h"


typedef enum {
    DISCONNECTED = 0,
    CONNECTED,
    CONNECTING,
    NOT_FOUND,
} status_t;

struct statusDevice_st
{
    status_t wifi;
    status_t sdcard;
    status_t ds3231Module;
    status_t bme280Sensor;
    status_t pms7003Sensor;
    status_t mqttClient;
};

struct moduleError_st
{
    uint64_t timestamp;
    esp_err_t sdError;
    esp_err_t ds3231Error;
    esp_err_t bme280Error;
    esp_err_t pms7003Error;
};


#endif
