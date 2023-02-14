#ifndef __DATAMANAGER_H__
#define __DATAMANAGER_H__

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <string.h>

struct dataSensor_st
{
    uint64_t timeStamp;

    float temperature;
    float humidity;
    float pressure;

    uint32_t pm1_0;
    uint32_t pm2_5;
    uint32_t pm10;

#if(0)
    uint32_t CO2;
    uint32_t NO;
    uint32_t CO;
    uint32_t SO2;
#endif
};

const char dataSensor_templateSaveToSDCard[] = "%s,%0.2f,%0.2f,%0.2f,%d,%d,%d";


#endif