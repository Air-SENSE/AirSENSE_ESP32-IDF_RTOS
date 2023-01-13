<<<<<<< HEAD
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

#include "time.h"
#include <esp_err.h>




#endif
=======
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

#include "config.h"
#include "time.h"
#include <esp_err.h>

struct {
    uint16_t pm_1_0_UG;
    uint16_t pm_2_5_UG;
    uint16_t pm_10_0_UG;

    float temperature;
    float humidity;
    float pressure;

    struct tm time;
} datacore;



/**
 * @brief Initialize devices
 * 
 * @return ESP_OK on success
 */
esp_err_t initDevices();



#endif
>>>>>>> 4093a5f80c80c982922bae844b0dc8c0c88f9ab3
