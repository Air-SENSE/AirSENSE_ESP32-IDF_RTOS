/*
 * SPDX-FileCopyrightText: 2017-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <esp_http_client.h>
#include <bootloader_common.h>
#include "esp_app_desc.h"
#include <sdkconfig.h>
#include "esp_https_ota.h"
#include "esp_event.h"
#include "esp_partition.h"

#ifdef __cplusplus
extern "C"
{
#endif

    esp_err_t esp_https_ota_begin_with_api_key(const esp_https_ota_config_t *ota_config, esp_https_ota_handle_t *handle, char *api_key);

    /**
     * @brief    Read image data from HTTP stream and write it to OTA partition
     *
     * This function reads image data from HTTP stream and writes it to OTA partition. This function
     * must be called only if esp_https_ota_begin() returns successfully.
     * This function must be called in a loop since it returns after every HTTP read operation thus
     * giving you the flexibility to stop OTA operation midway.
     *
     * @param[in]  https_ota_handle  pointer to esp_https_ota_handle_t structure
     *
     * @return
     *    - ESP_ERR_HTTPS_OTA_IN_PROGRESS: OTA update is in progress, call this API again to continue.
     *    - ESP_OK: OTA update was successful
     *    - ESP_FAIL: OTA update failed
     *    - ESP_ERR_INVALID_ARG: Invalid argument
     *    - ESP_ERR_INVALID_VERSION: Invalid chip revision in image header
     *    - ESP_ERR_OTA_VALIDATE_FAILED: Invalid app image
     *    - ESP_ERR_NO_MEM: Cannot allocate memory for OTA operation.
     *    - ESP_ERR_FLASH_OP_TIMEOUT or ESP_ERR_FLASH_OP_FAIL: Flash write failed.
     *    - For other return codes, refer OTA documentation in esp-idf's app_update component.
     */

#ifdef __cplusplus
}
#endif
