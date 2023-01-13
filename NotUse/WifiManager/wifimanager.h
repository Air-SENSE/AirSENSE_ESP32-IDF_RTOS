/**
 * @file wifimanager.h
 * @author Nguyen Nhu Hai Long ( @long27032002 )
 * @brief Wifi manager.
 * @version 0.1
 * @date 2022-12-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __WIFIMANAGER_H__
#define __WIFIMANAGER_H__

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


//typedef void (* WIFI_eventHandler_cb)(void*, esp_event_base_t, int32_t, void*)

// /**
//  * @brief 
//  * 
//  * @param argument 
//  * @param event_base 
//  * @param event_id 
//  * @param event_data 
//  * 
//  * @return void
//  */
// static void WIFI_eventHandler(void* argument, esp_event_base_t event_base, int32_t event_id, void* event_data);

//void WIFI_eventHandlerFunctionCallback(void* argument_t, esp_event_base_t event_base_t, int32_t event_id_t, void* event_data_t);

/**
 * @brief 
 * 
 * @param WIFI_ssid 
 * @param WIFI_password 
 * 
 * @return esp_err_t
 */
esp_err_t WIFI_initSTA(const char *WIFI_ssid, const char *WIFI_password, EventGroupHandle_t *WIFI_eventGroupHandle_t, esp_event_handler_t WIFI_eventHandle_t);


#endif
