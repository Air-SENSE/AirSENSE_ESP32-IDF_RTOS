<<<<<<< HEAD
#include "sdcard.h"

static const char *TAG = "SDcard";


esp_err_t sdcard_initialize(esp_vfs_fat_sdmmc_mount_config_t *_mount_config, sdmmc_card_t *_sdcard,
                            sdmmc_host_t *_host, spi_bus_config_t *_bus_config, sdspi_device_config_t *_slot_config)
{
    esp_err_t err_code;
    ESP_LOGI(__func__, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(__func__, "Using SPI peripheral");

    err_code = spi_bus_initialize(_host->slot, _bus_config, SPI_DMA_CHAN);
    if (err_code != ESP_OK)
    {
        ESP_LOGE(__func__, "Failed to initialize bus.");
        ESP_LOGE(__func__, "Failed to initialize the SDcard.");
        return ESP_ERROR_SD_INIT_FAILED;
    }
    _slot_config->gpio_cs = CONFIG_PIN_NUM_CS;
    _slot_config->host_id = _host->slot;

    ESP_LOGI(__func__, "Mounting filesystem");
    err_code = esp_vfs_fat_sdspi_mount(mount_point, _host, _slot_config, _mount_config, &_sdcard);
    
    if (err_code != ESP_OK) {
        if (err_code == ESP_FAIL) {
            ESP_LOGE(__func__, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(__func__, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err_code));
        }
        return err_code;
    }
    ESP_LOGI(__func__, "SDCard has been initialized.");
    ESP_LOGI(__func__, "Filesystem mounted");

    // Card has been initialized, print its properties
    ESP_LOGI(__func__, "SDCard properties.");
    sdmmc_card_print_info(stdout, _sdcard);
    return ESP_OK;
}


esp_err_t sdcard_writeDataToFile(const char *nameFile, const char *format, ...)
{
    char pathFile[64];
    sprintf(pathFile, "%s/%s.txt", mount_point, nameFile);

    ESP_LOGI(__func__, "Opening file %s...", pathFile);
    FILE *file = fopen(pathFile, "a");
    if (file == NULL)
    {
        ESP_LOGE(__func__, "Failed to open file for writing.");
        return ESP_ERROR_SD_OPEN_FILE_FAILED;
    }
    
    char *dataString;
    int lenght;
    va_list argumentsList;
    va_list argumentsList_copy;
    va_start(argumentsList, format);
    va_copy(argumentsList_copy, argumentsList);
    lenght = vsnprintf(NULL, 0, format, argumentsList_copy);
    va_end(argumentsList_copy);

    dataString = (char*)malloc(++lenght);
    if(dataString == NULL) {
        ESP_LOGE(TAG, "Failed to create string data for writing.");
        va_end(argumentsList);
        return ESP_FAIL;
    }

    vsnprintf(dataString, (++lenght), format, argumentsList);
    ESP_LOGI(TAG, "Success to create string data(%d) for writing.", lenght);
    ESP_LOGI(TAG, "Writing data to file %s...", pathFile);
    ESP_LOGI(TAG, "%s;\n", dataString);

    int returnValue = 0;
    returnValue = fprintf(file, "%s,", dataString);
    if (returnValue < 0)
    {
        ESP_LOGE(__func__, "Failed to write data to file %s.", pathFile);
        return ESP_ERROR_SD_WRITE_DATA_FAILED;
    }
    ESP_LOGI(__func__, "Success to write data to file %s.", pathFile);
    fclose(file);
    va_end(argumentsList);
    free(dataString);
    return ESP_OK;
}


esp_err_t sdcard_read_data_from_file(const char *nameFile, const char *format, ...)
{
    char pathFile[64];
    sprintf(pathFile, "%s/%s.txt", mount_point, nameFile);

    ESP_LOGI(__func__, "Opening file %s...", pathFile);
    FILE *file = fopen(pathFile, "r");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading.");
        return ESP_ERROR_SD_OPEN_FILE_FAILED;
    }

    // Read a string data from file
    char dataStr[256];
    char *returnPtr;
    returnPtr = fgets(dataStr, sizeof(dataStr), file);
    fclose(file);
    
    if (returnPtr == NULL)
    {
        ESP_LOGE(__func__, "Failed to read data from file %s.", pathFile);
        return ESP_ERROR_SD_READ_DATA_FAILED;
    }

    va_list argumentsList;
    va_start(argumentsList, format);
    int returnValue = 0;
    returnValue = vsscanf(dataStr, format, argumentsList);
    va_end(argumentsList);

    if (returnValue < 0)
    {
        ESP_LOGE(__func__, "Failed to read data from file %s.", pathFile);
        return ESP_ERROR_SD_READ_DATA_FAILED;
    }
    
    return ESP_OK;
}


esp_err_t sdcard_deinitialize(const char* _mount_point, sdmmc_card_t *_sdcard, sdmmc_host_t *_host)
{
    ESP_LOGI(__func__, "Deinitializing SD card...");
    // Unmount partition and disable SPI peripheral.
    ESP_ERROR_CHECK(esp_vfs_fat_sdcard_unmount(_mount_point, _sdcard));
    ESP_LOGI(__func__, "Card unmounted.");

    //deinitialize the bus after all devices are removed
    ESP_ERROR_CHECK(spi_bus_free(_host->slot));
    return ESP_OK;
}
=======
#include "sdcard.h"

esp_err_t sdcard_initialize(esp_vfs_fat_sdmmc_mount_config_t *_mount_config, sdmmc_card_t *_sdcard,
                            sdmmc_host_t *_host, spi_bus_config_t *_bus_config, sdspi_device_config_t *_slot_config)
{
    esp_err_t err_code;
    ESP_LOGI(TAG, "Initializing SD card");
    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    *_host = SDSPI_HOST_DEFAULT();
    err_code = spi_bus_initialize(_host->slot, bus_config, SPI_DMA_CHAN);
    if (err_code != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        ESP_LOGE(TAG, "Failed to initialize the SDcard.");
        return ESP_ERR_SD_INIT_FAILED;
    }
    _slot_config->gpio_cs = PIN_NUM_CS;
    _slot_config->host_id = _host->slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    err_code = esp_vfs_fat_sdspi_mount(mount_point, _host, _slot_config, _mount_config, _sdcard);
    
    if (err_code != ESP_OK) {
        if (err_code == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err_code));
        }
        return err_code;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, _sdcard);
    return ESP_OK;
}


esp_err_t sdcard_write_data_to_file(const char *nameFile, const char *format, ...)
{
    char pathFile[64];
    sprintf(pathFile, "%s/%s.txt", mount_point, nameFile);

    ESP_LOGI(TAG, "Opening file %s...", pathFile);
    FILE *file = fopen(pathFile, "a");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing.");
        return ESP_ERR_SD_OPEN_FILE_FAILED;
    }
    
    char *dataString = "";
    int lenght;
    va_list argumentsList;
    va_list argumentsList_copy;
    va_start(argumentsList, format);
    va_copy(copy, argumentsList);
    lenght = vsnprintf(NULL, 0, format, argumentsList_copy);
    va_end(argumentsList_copy);

    dataString = (char*)malloc(++lenght);
    if(dataString == NULL) {
        ESP_LOGE(TAG, "Failed to create string data for writing.");
        va_end(argumentsList);
        return ESP_FAIL;
    }

    vsnprintf(dataString, (++lenght), format, argumentsList);
    ESP_LOGI(TAG, "Success to create string data(%d) for writing.", lenght);
    ESP_LOGI(TAG, "Writing data to file %s...", pathFile);
    ESP_LOGI(TAG, "%s;\n", dataString);

    int returnValue = 0;
    returnValue = fprintf(file, "%s,", dataString);
    if (returnValue < 0)
    {
        ESP_LOGE(TAG, "Failed to write data to file %s.", pathFile);
        return ESP_ERR_SD_WRITE_DATA_FAILED;
    }
    ESP_LOGE(TAG, "Success to write data to file %s.", pathFile);
    fclose(file);
    va_end(argumentsList);
    free(dataString);
    return ESP_OK;
}


esp_err_t sdcard_read_data_from_file(const char *nameFile, const char *format, ...)
{
    char pathFile[64];
    sprintf(pathFile, "%s/%s.txt", mount_point, nameFile);

    ESP_LOGI(TAG, "Opening file %s...", pathFile);
    FILE *file = fopen(pathFile, "r");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading.");
        return ESP_ERR_SD_OPEN_FILE_FAILED;
    }

    // Read a string data from file
    char calibDataStr[256];
    char *returnPtr;
    returnPtr = fgets(calibDataStr, sizeof(calibDataStr), file);
    if (returnPtr == NULL)
    {
        ESP_LOGE(TAG, "Failed to read data from file %s.", pathFile);
        return ESP_ERR_SD_READ_DATA_FAILED;
    }

    // Strip newline
    char *pos = strchr(calibDataStr, '\n');
    if (pos) {*pos = '\0';}

    va_list argumentsList;
    va_start(argumentsList, format);
    int returnValue = 0;
    returnValue = vsscanf(fileContent_string, format, argumentsList);
    va_end(argumentsList);

    if (returnValue < 0)
    {
        ESP_LOGE(TAG, "Failed to read data from file %s.", pathFile);
        return ESP_ERR_SD_READ_DATA_FAILED;
    }
    
    fclose(f);
    return ESP_OK;
}


esp_err_t sdcard_deinitialize(const char* _mount_point, sdmmc_card_t *_sdcard, sdmmc_host_t *_host)
{
    ESP_LOGI(TAG, "Deinitializing SD card.");
    // Unmount partition and disable SPI peripheral.
    esp_vfs_fat_sdcard_unmount(_mount_point, _sdcard);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(_host->slot);
    return ESP_OK;
}
>>>>>>> 4093a5f80c80c982922bae844b0dc8c0c88f9ab3
