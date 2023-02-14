/**
 * @file sdcard.h
 * @author Nguyen Nhu Hai Long ( @long27032002 )
 * @brief 
 * @version 0.1
 * @date 2022-11-29
 * @copyright Copyright (c) 2022
 */
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include <string.h>
#include <stdarg.h>
#include <esp_log.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define ID_SD_CARD 0x01

#define SPI_DMA_CHAN    1
#define ESP_ERROR_SD_INIT_FAILED            ((ID_SD_CARD << 12)|(0x00))
#define ESP_ERROR_SD_OPEN_FILE_FAILED       ((ID_SD_CARD << 12)|(0x01))
#define ESP_ERROR_SD_WRITE_DATA_FAILED      ((ID_SD_CARD << 12)|(0x02))
#define ESP_ERROR_SD_READ_DATA_FAILED       ((ID_SD_CARD << 12)|(0x03))
#define ESP_ERROR_SD_RENAME_FILE_FAILED     ((ID_SD_CARD << 12)|(0x04))

// #define PIN_NUM_MISO 21
// #define PIN_NUM_MOSI 19
// #define PIN_NUM_CLK  18
// #define PIN_NUM_CS   5

#define SPI_BUS_CONFIG_DEFAULT()  { .mosi_io_num = CONFIG_PIN_NUM_MOSI,    \
                                    .miso_io_num = CONFIG_PIN_NUM_MISO,    \
                                    .sclk_io_num = CONFIG_PIN_NUM_CLK,     \
                                    .quadwp_io_num = -1,            \
                                    .quadhd_io_num = -1,            \
                                    .max_transfer_sz = 4000,        \
}

#define MOUNT_CONFIG_DEFAULT()    { .format_if_mount_failed = true,         \
                                    .max_files = 5,                         \
                                    .allocation_unit_size = (1024 * 1024),  \
}

#define MOUNT_POINT "/sdcard"
static const char mount_point[] = MOUNT_POINT;


/**
 * @brief Initializes SD card with configuration.
 * 
 * @param[in] _mount_config Pointer to structure with extra parameters for mounting FATFS.
 * @param[in] _sdcard       Pointer SD/MMC card information structure.
 * @param[in] _host         Pointer to structure describing SDMMC host. This structure can be
 *                          initialized using SDSPI_HOST_DEFAULT() macro.
 * @param[in] _bus_config   Pointer to configuration structure for a SPI bus.
 * @param[in] _slot_config  Pointer to structure with slot configuration.
 *                          For SPI peripheral, pass a pointer to sdspi_device_config_t
 *                          structure initialized using SDSPI_DEVICE_CONFIG_DEFAULT().
 * 
 * @return esp_err_t 
 * 
 * @retval ESP_OK on success.
 * @retval ESP_FAIL on fail.
 */
esp_err_t sdcard_initialize(esp_vfs_fat_sdmmc_mount_config_t *_mount_config, sdmmc_card_t *_sdcard,
                            sdmmc_host_t *_host, spi_bus_config_t *_bus_config, sdspi_device_config_t *_slot_config);


/**
 * @brief Write data to file follow format.
 * 
 * @param[in] nameFile Name file.
 * @param[in] format Template structure for data store.
 * @param ... #__VA_ARGS__ : List arguments
 * 
 * @return esp_err_t 
 * 
 * @retval  - ESP_OK on success.
 * @retval  - ESP_FAIL on fail.
 * @retval  - ESP_ERROR_SD_OPEN_FILE_FAILED on can't open file.
 * @retval  - ESP_ERROR_SD_WRITE_DATA_FAILED on fail to write data.
 */
esp_err_t sdcard_writeDataToFile(const char *nameFile, const char *format, ...);


/**
 * @brief Read data to file follow format.
 * 
 * @param nameFile Name file.
 * @param format Template structure for data store.
 * @param ... #__VA_ARGS__ : List arguments
 * 
 * @return esp_err_t 
 * 
 * @retval  - ESP_OK on success.
 * @retval  - ESP_FAIL on fail.
 * @retval  - ESP_ERROR_SD_OPEN_FILE_FAILED on can't open file.
 * @retval  - ESP_ERROR_SD_READ_DATA_FAILED on fail to read data.
 */
esp_err_t sdcard_readDataToFile(const char *nameFile, const char *format, ...);


/**
 * @brief Initializes SD card and SPI bus
 * 
 * @param _mount_point Pointer to structure with extra parameters for mounting FATFS.
 * @param _sdcard Pointer SD/MMC card information structure.
 * @param _host Pointer to structure describing SDMMC host. This structure can be
 *              initialized using SDSPI_HOST_DEFAULT() macro.
 * 
 * @return esp_err_t 
 * 
 * @retval  - ESP_OK on success.
 */
esp_err_t sdcard_deinitialize(const char* _mount_point, sdmmc_card_t *_sdcard, sdmmc_host_t *_host);


esp_err_t sdcard_renameFile(const char *oldNameFile, char *newNameFile);

#endif
