#ifndef __SNTP_SYNC_H__
#define __SNTP_SYNC_H__

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sntp.h"

#define RETRY_SET_TIME_SYSTEM   50

/**
 * @brief Initialize SNTP
 * 
 * @return esp_err_t 
 */
void sntp_init_func();

/**
 * @brief 
 * 
 * @param timeInfo 
 * @param time 
 * @return esp_err_t 
 */
esp_err_t sntp_setTime(struct tm *timeInfo, time_t *time);

#endif