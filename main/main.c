<<<<<<< HEAD
/**
 * @file main.c
 * @author Nguyen Nhu Hai Long ( @long27032002 )
 * @brief Main file of AirSENSE project
 * @version 0.1
 * @date 2023-01-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_spi_flash.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "bme280.h"
#include "sdcard.h"
#include "button.h"
#include "pms7003.h"
#include "DS3231Time.h"
#include "Tasks.h"
//#include "DeviceManager.h"

// #define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

//static const char *TAG = "Main";

static EventGroupHandle_t WIFI_eventGroup;

static int WIFI_retryCount = 0;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static void WIFI_eventHandler(void* argument, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (WIFI_retryCount < CONFIG_MAXIMUM_RETRY) {
            esp_wifi_connect();
            WIFI_retryCount++;
            ESP_LOGI(__func__, "Retry to connect to the AP...");
        } else {
            xEventGroupSetBits(WIFI_eventGroup, WIFI_FAIL_BIT);
        }
        ESP_LOGI(__func__, "Connect to the AP fail.");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(__func__, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        WIFI_retryCount = 0;
        xEventGroupSetBits(WIFI_eventGroup, WIFI_CONNECTED_BIT);
    }
}

void WIFI_initSTA()
{
    WIFI_eventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t WIFI_initConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&WIFI_initConfig));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &WIFI_eventHandler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &WIFI_eventHandler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_SSID,
            .password = CONFIG_PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(__func__, "WIFI initialize STA finished.");

    EventBits_t bits = xEventGroupWaitBits(WIFI_eventGroup,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            2000 / portTICK_RATE_MS);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(__func__, "Connected to ap SSID:%s password:%s",
                 CONFIG_SSID, CONFIG_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(__func__, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_SSID, CONFIG_PASSWORD);
    } else {
        ESP_LOGE(__func__, "UNEXPECTED EVENT");
    }
}


static void initialize_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
}



void app_main(void)
{
    // Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(200)); 

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI("ESP_info", "This is %s chip with %d CPU core(s), revision %d, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            chip_info.revision,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI("ESP_info", "silicon revision %d, ", chip_info.revision);
    ESP_LOGI("ESP_info", "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGI("ESP_info", "Minimum free heap size: %d bytes\r\n", esp_get_minimum_free_heap_size());

    // Booting
    ESP_LOGI(__func__, "Booting....");
    ESP_LOGI(__func__, "Name device: %s.", CONFIG_NAME_DEVICE);
    ESP_LOGI(__func__, "Firmware version %s.", CONFIG_FIRMWARE_VERSION);

    // Initialize nvs partition
    ESP_LOGI(__func__, "Initialize nvs partition.");
    initialize_nvs();

// Initialize SD card
#if(CONFIG_USING_SDCARD)
    // Initialize SPI Bus

    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t  mount_config_t   = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t            spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t                host_t           = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t       slot_config      = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));
#endif  //CONFIG_USING_SDCARD

#if(CONFIG_USING_WIFI)
    WIFI_initSTA();
#endif

// Initialize RTC module
#if(CONFIG_USING_RTC)
    // Initialize I2C wire for RTC module
    // i2c_config_t ds3231_i2cConfig = {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = CONFIG_RTC_PIN_NUM_SDA,
    //     .scl_io_num = CONFIG_RTC_PIN_NUM_SCL,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = CONFIG_RTC_I2C_FREQ_HZ,
    // };
    // i2c_param_config(CONFIG_RTC_I2C_PORT, &ds3231_i2cConfig);
    // ESP_ERROR_CHECK(i2c_driver_install(CONFIG_RTC_I2C_PORT, ds3231_i2cConfig.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
    
    ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);
    i2c_dev_t i2c_rtcDevice;
    memset(&i2c_rtcDevice, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&i2c_rtcDevice, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
#endif  //CONFIG_USING_RTC

// Initialize BME280 Sensor
#if(CONFIG_USING_BME280)
    // Initialize I2C wire for BME280 Sensor
    // i2c_config_t bme280_i2cConfig = {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = CONFIG_BME_PIN_NUM_SDA,
    //     .scl_io_num = CONFIG_BME_PIN_NUM_SCL,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = CONFIG_BME_I2C_FREQ_HZ,
    // };
    // i2c_param_config(CONFIG_BME_I2C_PORT, &bme280_i2cConfig);
    // ESP_ERROR_CHECK(i2c_driver_install(CONFIG_BME_I2C_PORT, bme280_i2cConfig.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    ESP_LOGI(__func__, "Initialize BME280 sensor(I2C/Wire%d).", CONFIG_BME_I2C_PORT);
    ESP_ERROR_CHECK(i2cdev_init());
    bmp280_t bme280_device;
    bmp280_params_t bme280_params;
    ESP_ERROR_CHECK(bme280_init(&bme280_device, &bme280_params, BME280_ADDRESS,
                                CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL));

#endif  //CONFIG_USING_BME280

#if(CONFIG_USING_PMS7003)
    uart_config_t pms_uart_config = UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(pms7003_initUart(&pms_uart_config));

#endif  //CONFIG_USING_PMS7003
