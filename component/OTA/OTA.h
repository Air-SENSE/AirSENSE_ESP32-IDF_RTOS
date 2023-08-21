#ifndef __OTA_H__
#define __OTA_H__

#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_efuse.h"
#include "esp_err.h"


extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


#define OTA_URL_SIZE 256

/**
 * @brief Validate new firmware that server hosting.
 * 
 * @param new_app_info: information of the downloaded firmware form server.
 * @return If new version of firmware is valid, return ESP_OK. 
 *          Else return ESP_FAIL. 
 */
esp_err_t firmware_validateVersion(esp_app_desc_t *new_app_info);

/**
 * @brief 
 * 
 * @param http_client 
 * @return esp_err_t 
 */
esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client);

/**
 * @brief Download OTA from server, validate it. If it's valid, then write it to ota_x area 
 *        Partition table of flash memory.
 * 
 * @param http_config: Configuration of http client that have firmware update url from server
 *                     and OTA receive timeout.
 * @return If downloaded firmware from server valid and writing it to flash memory 
 *         successful return ESP_OK. Else return ESP_FAIL.
 */
esp_err_t updateOTA(esp_http_client_config_t *http_config);

/**
 * @brief 
 * 
 * @return esp_err_t 
 */
esp_err_t bootloaderApp_rollback(void);

#endif
