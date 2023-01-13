#ifndef __DATAMANAGER_H__
#define __DATAMANAGER_H__

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

struct {
    unsigned long timeStamp;

    float temperature;
    float humidity;
    float pressyre;

    uint32_t pm1_0;
    uint32_t pm2_5;
    uint32_t pm10;

#if(0)
    uint32_t CO2;
    uint32_t NO;
    uint32_t CO;
    uint32_t SO2;
#endif
} dataSensor;


#endif