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

/*------------------------------------ INCLUDE LIBRARY ------------------------------------ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include <sys/param.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_cpu.h"
#include "esp_mem.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_attr.h"

#include <spi_flash_mmap.h>
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sys.h"
// Device and component libraries
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "https_ota.h"
#include "bme280.h"
#include "sdcard.h"
#include "pms7003.h"
#include "mhz14a.h"
#include "DS3231Time.h"
#include "datamanager.h"
#include "DeviceManager.h"
#include "sntp_sync.h"
#include "pcf8574.h"
// Display libraries
#include "lvgl.h"
#include "ili9341_extended.h"

/*------------------------------------ DEFINE ------------------------------------ */

__attribute__((unused)) static const char *TAG = "Main";

#define PERIOD_GET_DATA_FROM_SENSOR (TickType_t)(CONFIG_GET_DATA_PERIOD * 1000 / portTICK_PERIOD_MS)
#define PERIOD_SAVE_DATA_SENSOR_TO_SDCARD (TickType_t)(2500 / portTICK_PERIOD_MS)
#define PERIOD_SAVE_DATA_AFTER_WIFI_RECONNECT (TickType_t)(1000 / portTICK_PERIOD_MS)

#define NO_WAIT (TickType_t)(0)
#define WAIT_10_TICK (TickType_t)(10 / portTICK_PERIOD_MS)
#define WAIT_100_TICK (TickType_t)(100 / portTICK_PERIOD_MS)

#define QUEUE_SIZE 10U
#define NAME_FILE_QUEUE_SIZE 5U

static EventGroupHandle_t fileStore_eventGroup;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_CONNECT_FAIL_BIT BIT1
#define WIFI_AVAIABLE_BIT BIT0
#define MQTT_CLIENT_CONNECTED WIFI_AVAIABLE_BIT
#define WIFI_DISCONNECT_BIT BIT1
#define MQTT_CLIENT_DISCONNECTED WIFI_DISCONNECT_BIT
#define FILE_RENAME_NEWDAY BIT2
#define FILE_RENAME_FROMSYNC BIT3

#define PCF8574_ADDR 0x27

// Task handle
TaskHandle_t getDataFromSensorTask_handle = NULL;
TaskHandle_t saveDataSensorToSDcardTask_handle = NULL;
TaskHandle_t saveDataSensorAfterReconnectWiFiTask_handle = NULL;
TaskHandle_t mqttPublishMessageTask_handle = NULL;
TaskHandle_t sntpGetTimeTask_handle = NULL;
TaskHandle_t allocateDataToMQTTandSDQueue_handle = NULL;
TaskHandle_t ota_firmware_update_handle = NULL;
TaskHandle_t ota_reset_time_handle = NULL;
TaskHandle_t display_lcd_tft_handle = NULL;

// Semaphore
SemaphoreHandle_t getDataSensor_semaphore = NULL;
SemaphoreHandle_t writeDataToSDcard_semaphore = NULL;
SemaphoreHandle_t sentDataToMQTT_semaphore = NULL;
SemaphoreHandle_t debug_semaphore = NULL;
SemaphoreHandle_t writeDataToSDcardNoWifi_semaphore = NULL;
SemaphoreHandle_t allocateDataToMQTTandSDQueue_semaphore = NULL;
SemaphoreHandle_t xOTA_reset_timer_Semaphore = NULL;
SemaphoreHandle_t xOTASemaphore = NULL;

#define ESP_INR_FLAG_DEFAULT 0
#define SW_1 36       // Negative edge
#define SW_2 39       // Negative edge
#define SW_BUILT_IN 0 // Negative edge
#define BACKLIGHT_LCD_GPIO CONFIG_LCD_PIN_NUM_BK_LIGHT

gptimer_handle_t OTA_Timer = NULL; // Timer for OTA

QueueHandle_t dataSensorSentToSD_queue;
QueueHandle_t dataSensorSentToMQTT_queue;
QueueHandle_t moduleError_queue;
QueueHandle_t dateTimeLostWiFi_queue;
QueueHandle_t dataSensor_queue;
QueueHandle_t lcdDisplay_queue;

static struct statusDevice_st statusDevice = {0};

static esp_mqtt_client_handle_t mqttClient_handle = NULL;
static char CurrentFileNametoSaveData[21]; // file name không được chứa kí tự đặc biệt
static uint8_t Debug_Led_status = 0xFF;
uint8_t MAC_addressArray[6];

// Whether send data to MQTT queue or not (depend on WIFI connection)
bool sendToMQTTQueue = false;
const char *formatDataSensorString = "{\n\t\"device_id\":\"%x%x%x%x\",\n\t\"Time\":%llu,\n\t\"Temperature\":%.2f,\n\t\"Humidity\":%.2f,\n\t\"Pressure\":%.2f,\n\t\"PM1\":%" PRIu32 ",\n\t\"PM2.5\":%" PRIu32 ",\n\t\"PM10\":%" PRIu32 ",\n\t\"CO2\":%" PRIu32 "\n}";

//------------------------------------------------------------------

i2c_dev_t ds3231_device;
i2c_dev_t pcf8574_device;

bmp280_t bme280_device;
bmp280_params_t bme280_params;

uart_config_t pms_uart_config = UART_CONFIG_DEFAULT();           // Configuration for PMS7003 uart port
uart_config_t mhz14a_uart_config = MHZ14A_UART_CONFIG_DEFAULT(); // Configuration for MH-Z14A uart port

lv_disp_t *lcd_mainscreen;
volatile bool is_LCD_turned_off = false; // define true if the screen is turned off
extern void ui_airsense_init(lv_disp_t *disp);
extern void airsense_wifi_state_display(bool wifi_state);
extern void airsense_sdcard_state_display(bool sdcard_state);
extern void airsense_sensor_data_display(struct dataSensor_st sensor_data);
extern void airsense_date_time_display(char *date_time_value);

static void mqtt_app_start(void);
static void sntp_app_start(void);
void sendDataSensorToMQTTServerAfterReconnectWiFi_task(void *parameters);

// Interrupt function for turning on or off screen
void IRAM_ATTR on_off_lcd_handler(void *arg)
{
    if (is_LCD_turned_off == false)
    {
        is_LCD_turned_off = true;
        xQueueReset(lcdDisplay_queue); // Reset the display queue to its empty state
        gpio_set_level(BACKLIGHT_LCD_GPIO, LCD_BK_LIGHT_OFF_LEVEL);
    }
    else
    {
        if (eTaskGetState(display_lcd_tft_handle) == eSuspended)
        {
            is_LCD_turned_off = false;
            gpio_set_level(BACKLIGHT_LCD_GPIO, LCD_BK_LIGHT_ON_LEVEL);
            xTaskResumeFromISR(display_lcd_tft_handle);
        }
    }
}

/*------------------------------------ OTA ------------------------------------ */
#if (CONFIG_USING_FIRMWARE_UPDATE_BY_OTA)
void IRAM_ATTR OTA_timer_isr_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    if (ota_reset_time_handle != NULL)
    {
        xSemaphoreGiveFromISR(xOTA_reset_timer_Semaphore, xHigherPriorityTaskWoken);
    }
    xSemaphoreGiveFromISR(xOTASemaphore, xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ota_reset_timer(void *pvParameter)
{
    if (xSemaphoreTake(xOTA_reset_timer_Semaphore, portMAX_DELAY) == pdPASS)
    {
        uint64_t seconds_until_new_Day = SECONDS_PER_DAY * 1000;
        if (OTA_Timer != NULL)
        {
            gptimer_set_raw_count(OTA_Timer, 0);
            gptimer_alarm_config_t OTA_alarm_config = {
                .alarm_count = seconds_until_new_Day * 1000, // period = 1s
                .reload_count = 0,
                .flags.auto_reload_on_alarm = true,
            };

            ESP_ERROR_CHECK(gptimer_stop(OTA_Timer));
            ESP_ERROR_CHECK(gptimer_set_alarm_action(OTA_Timer, &OTA_alarm_config));
            ESP_ERROR_CHECK(gptimer_start(OTA_Timer));
        }
        vTaskDelete(NULL);
    }
}

static void OTA_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == ESP_HTTPS_OTA_EVENT)
    {
        switch (event_id)
        {
        case ESP_HTTPS_OTA_START:
            ESP_LOGI(TAG, "OTA started");
            break;
        case ESP_HTTPS_OTA_CONNECTED:
            ESP_LOGI(TAG, "Connected to server");
            break;
        case ESP_HTTPS_OTA_GET_IMG_DESC:
            ESP_LOGI(TAG, "Reading Image Description");
            break;
        case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
            ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
            break;
        case ESP_HTTPS_OTA_DECRYPT_CB:
            ESP_LOGI(TAG, "Callback to decrypt function");
            break;
        case ESP_HTTPS_OTA_WRITE_FLASH:
            ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
            break;
        case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
            ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
            break;
        case ESP_HTTPS_OTA_FINISH:
            ESP_LOGI(TAG, "OTA finish");
            break;
        case ESP_HTTPS_OTA_ABORT:
            ESP_LOGI(TAG, "OTA abort");
            break;
        }
    }
}

static esp_err_t validate_firmware(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
    {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }

    if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0)
    {
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}

void ota_firmware_update_task(void *pvParameter)
{
    for (;;)
    {
        if (xSemaphoreTake(xOTASemaphore, portMAX_DELAY) == pdPASS)
        {
            Debug_Led_status &= 0xF7;
            pcf8574_port_write(&pcf8574_device, Debug_Led_status);
            ESP_LOGI(TAG, "Starting checking new firmware availability");
            char API[100];
            sprintf(API, "api_key=%s", CONFIG_API_KEY);
            esp_err_t ota_finish_err = ESP_OK;
            esp_http_client_config_t config = {
                .url = CONFIG_FIRMWARE_UPDATE_URL,
                .cert_pem = NULL,
                .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
                .keep_alive_enable = true,
            };

            esp_https_ota_config_t ota_config = {
                .http_config = &config,
                .http_client_init_cb = _http_client_init_cb,
            };

            esp_https_ota_handle_t https_ota_handle = NULL;
            esp_err_t err = esp_https_ota_begin_with_api_key(&ota_config, &https_ota_handle, API);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "ESP HTTPS OTA Begin failed");
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                goto ota_end;
            }

            esp_app_desc_t app_desc;
            err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_https_ota_read_img_desc failed");
                goto ota_end;
            }
            err = validate_firmware(&app_desc);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "image header verification failed");
                goto ota_end;
            }
            ESP_LOGI(TAG, "New firmware version is available. The updating is being in progress");
            vTaskSuspend(getDataFromSensorTask_handle);
            vTaskSuspend(allocateDataToMQTTandSDQueue_handle);
            while (1)
            {
                err = esp_https_ota_perform(https_ota_handle);
                if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
                {
                    break;
                }
                ESP_LOGD(TAG, "Image bytes read: %d", esp_https_ota_get_image_len_read(https_ota_handle));
            }

            if (esp_https_ota_is_complete_data_received(https_ota_handle) != true)
            {
                ESP_LOGE(TAG, "Data was not completely received.");
            }
            else
            {
                ota_finish_err = esp_https_ota_finish(https_ota_handle);
                ESP_LOGE(TAG, "%s", esp_err_to_name(ota_finish_err));
                if ((err == ESP_OK) && (ota_finish_err == ESP_OK))
                {
                    ESP_LOGI(TAG, "Updating firmware with OTA is successful. Rebooting your device in few seconds ...");
                    Debug_Led_status |= 0x08;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Debug_Led_status &= 0xF7;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Debug_Led_status |= 0x08;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Debug_Led_status &= 0xF7;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Debug_Led_status |= 0x08;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Debug_Led_status &= 0xF7;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    Debug_Led_status |= 0x08;
                    pcf8574_port_write(&pcf8574_device, Debug_Led_status);
                    esp_restart();
                }
                else
                {
                    if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED)
                    {
                        ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                    }
                    ESP_LOGE(TAG, "Firmware update failed 0x%x", ota_finish_err);
                    goto ota_end_2;
                }
            }
        ota_end_2:
            xTaskResumeAll();
        ota_end:
            Debug_Led_status |= 0x08;
            pcf8574_port_write(&pcf8574_device, Debug_Led_status);
            esp_https_ota_abort(https_ota_handle);
            ESP_LOGE(TAG, "OTA firmware update update failed. Try to reset to check if the update is available");
        }
    }
}
#endif
/*------------------------------------ WIFI ------------------------------------ */
bool is_previous_Wifi_connected = true;
static void WiFi_eventHandler(void *argument, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(__func__, "Trying to connect with Wi-Fi...\n");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(__func__, "Wi-Fi connected AP SSID:%s password:%s.\n", CONFIG_SSID, CONFIG_PASSWORD);
            Debug_Led_status &= 0xFE;
            pcf8574_port_write(&pcf8574_device, Debug_Led_status);
            break;
            /* When esp32 disconnect to wifi, this event become
             * active. We suspend the task mqttPublishMessageTask. */
        case WIFI_EVENT_STA_DISCONNECTED:

            ESP_LOGI(__func__, "Wi-Fi disconnected: Retrying connect to AP SSID:%s password:%s", CONFIG_SSID, CONFIG_PASSWORD);
            Debug_Led_status |= 0x01;
            pcf8574_port_write(&pcf8574_device, Debug_Led_status);
            if (mqttPublishMessageTask_handle != NULL && eTaskGetState(mqttPublishMessageTask_handle) != eSuspended)
            {
                vTaskSuspend(mqttPublishMessageTask_handle);
                statusDevice.mqttClient = DISCONNECTED;
                xEventGroupSetBits(fileStore_eventGroup, MQTT_CLIENT_DISCONNECTED);
                sendToMQTTQueue = false;
                ESP_LOGI(__func__, "set bit disconnect");
            }
#if (CONFIG_USING_LCD_TFT)
            if ((is_LCD_turned_off == false) && (is_previous_Wifi_connected == true))
            {
                airsense_wifi_state_display(false);
            }
            is_previous_Wifi_connected = false;
#endif
            esp_wifi_connect();
            break;

        default:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            is_previous_Wifi_connected = true;
            if (is_LCD_turned_off == false)
            {
                airsense_wifi_state_display(true);
            }

            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

            ESP_LOGI(__func__, "Starting MQTT Client...\n");
#ifdef CONFIG_RTC_TIME_SYNC
            if (sntpGetTimeTask_handle == NULL)
            {
                sntp_app_start();
            }
#endif
#if (CONFIG_USING_FIRMWARE_UPDATE_BY_OTA)
            if (xOTASemaphore != NULL)
            {
                xTaskCreate(ota_firmware_update_task, "OTA_button_Task", 4096 * 5, NULL, 30, &ota_firmware_update_handle);
            }
            if (xOTA_reset_timer_Semaphore != NULL)
            {
                xTaskCreate(ota_reset_timer, "OTA_reset_timer_Task", 4096, NULL, 29, &ota_reset_time_handle);
            }
#endif
            /* When connect/reconnect wifi, esp32 take an IP address and this
             * event become active. If it's the first-time connection, create
             * task mqttPublishMessageTask, else resume that task. */
            if (mqttPublishMessageTask_handle == NULL)
            {
                mqtt_app_start();
                statusDevice.mqttClient = CONNECTED;
            }
            else
            {
                if (eTaskGetState(mqttPublishMessageTask_handle) == eSuspended)
                {
                    vTaskResume(mqttPublishMessageTask_handle);
                    ESP_LOGI(__func__, "Resume task mqttPublishMessageTask.");
                }
                if (saveDataSensorAfterReconnectWiFiTask_handle == NULL)
                {
                    // Create task to send data from sensor read by getDataFromSensor_task() to MQTT queue after reconnect to Wifi
                    // Period 100ms
                    xTaskCreate(sendDataSensorToMQTTServerAfterReconnectWiFi_task, "SendDataAfterReconnect", (1024 * 16), NULL, (UBaseType_t)15, &saveDataSensorAfterReconnectWiFiTask_handle);
                    if (saveDataSensorAfterReconnectWiFiTask_handle != NULL)
                    {
                        ESP_LOGW(__func__, "Create task reconnected OK.");
                    }
                    else
                    {
                        ESP_LOGE(__func__, "Create task reconnected failed.");
                    }
                }
                else if (eTaskGetState(saveDataSensorAfterReconnectWiFiTask_handle) == eSuspended)
                {
                    vTaskResume(saveDataSensorAfterReconnectWiFiTask_handle);
                    ESP_LOGI(__func__, "Resume task saveDataSensorAfterReconnectWiFi.");
                }
            }
        }
    }

    return;
}

/**
 * @brief This function initialize wifi and create, start WiFi handle such as loop (low priority)
 *
 */
void WIFI_initSTA(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t WIFI_initConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_init(&WIFI_initConfig));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_register(WIFI_EVENT,
                                                                      ESP_EVENT_ANY_ID,
                                                                      &WiFi_eventHandler,
                                                                      NULL,
                                                                      &instance_any_id));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_register(IP_EVENT,
                                                                      IP_EVENT_STA_GOT_IP,
                                                                      &WiFi_eventHandler,
                                                                      NULL,
                                                                      &instance_got_ip));

    static wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_SSID,
            .password = CONFIG_PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

    ESP_LOGI(__func__, "WIFI initialize STA finished.");
}

/*          -------------- MQTT --------------           */

/**
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param[in] handler_args user data registered to the event.
 * @param[in] base Event base for the handler(always MQTT Base in this example).
 * @param[in] event_id The id for the received event.
 * @param[in] event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(__func__, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(__func__, "MQTT_EVENT_CONNECTED");
        Debug_Led_status &= 0xFD;
        pcf8574_port_write(&pcf8574_device, Debug_Led_status);
        xEventGroupSetBits(fileStore_eventGroup, MQTT_CLIENT_CONNECTED);
        statusDevice.mqttClient = CONNECTED;
        sendToMQTTQueue = true;

        if (eTaskGetState(mqttPublishMessageTask_handle) == eSuspended)
        {
            vTaskResume(mqttPublishMessageTask_handle);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGE(__func__, "MQTT_EVENT_DISCONNECTED");
        Debug_Led_status |= 0x02;
        pcf8574_port_write(&pcf8574_device, Debug_Led_status);
        statusDevice.mqttClient = DISCONNECTED;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(__func__, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGE(__func__, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(__func__, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(__func__, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGE(__func__, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(__func__, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;

    default:
        ESP_LOGI(__func__, "Other event id:%d", event->event_id);
        break;
    }
}

/**
 * @brief Publish message dataSensor receive from dataSensorSentToMQTT_queue to MQTT
 *
 */
void mqttPublishMessage_task(void *parameters)
{
    // char *mqttTopic = malloc(strlen((char *)parameters) + 1);
    // int _size_mqttTopic = sizeof(mqttTopic);
    // memset(mqttTopic, 0, _size_mqttTopic);
    // memcpy(mqttTopic, (char *)parameters, _size_mqttTopic);
    sentDataToMQTT_semaphore = xSemaphoreCreateMutex();

    for (;;)
    {
        struct dataSensor_st dataSensorReceiveFromQueue;
        if (statusDevice.mqttClient == CONNECTED)
        {
            if (uxQueueMessagesWaiting(dataSensorSentToMQTT_queue) != 0)
            {
                if (xQueueReceive(dataSensorSentToMQTT_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) == pdPASS)
                {
                    ESP_LOGI(__func__, "Receiving data from queue successfully.");
                    if (xSemaphoreTake(sentDataToMQTT_semaphore, portMAX_DELAY) == pdTRUE)
                    {
                        esp_err_t error = ESP_OK;
                        WORD_ALIGNED_ATTR char mqttMessage[256];
                        sprintf(mqttMessage, formatDataSensorString, MAC_addressArray[0],
                                MAC_addressArray[1],
                                MAC_addressArray[2],
                                MAC_addressArray[3],
                                dataSensorReceiveFromQueue.timeStamp,
                                dataSensorReceiveFromQueue.temperature,
                                dataSensorReceiveFromQueue.humidity,
                                dataSensorReceiveFromQueue.pressure,
                                dataSensorReceiveFromQueue.pm1_0,
                                dataSensorReceiveFromQueue.pm2_5,
                                dataSensorReceiveFromQueue.pm10,
                                dataSensorReceiveFromQueue.CO2);

                        error = esp_mqtt_client_publish(mqttClient_handle, "AIRSENSE_PROJECT/DEVICE1", mqttMessage, 0, 0, 1);
                        xSemaphoreGive(sentDataToMQTT_semaphore);
                        if (error == ESP_FAIL)
                        {
                            ESP_LOGE(__func__, "MQTT client publish message failed...");
                        }
                        else
                        {
                            ESP_LOGI(__func__, "MQTT client publish message success.");
                        }
                    }
                    vTaskDelay((TickType_t)(1000 / portTICK_PERIOD_MS));
                }
            }
            else
            {
                vTaskDelay(PERIOD_GET_DATA_FROM_SENSOR);
            }
        }
        else
        {
            ESP_LOGE(__func__, "MQTT Client disconnected.");
            // Suspend ourselves.
            vTaskSuspend(NULL);
        }
    }
}

/**
 * @brief This function initialize MQTT client and create, start MQTT Client handle such as loop (low priority)
 *
 */
static void mqtt_app_start(void)
{
    WORD_ALIGNED_ATTR char MAC_address[32] = {0};
    esp_read_mac(MAC_addressArray, ESP_MAC_WIFI_STA); // Get MAC address of ESP32
    sprintf(MAC_address, "%" PRIx8 "%" PRIx8 "%" PRIx8 "%" PRIx8 "%" PRIx8 "%" PRIx8, MAC_address[0], MAC_address[1], MAC_address[2], MAC_address[3], MAC_address[4], MAC_address[5]);
    WORD_ALIGNED_ATTR char mqttTopic[64] = {0};
    WORD_ALIGNED_ATTR char mqttTopic_32bytes[32] = {0};
    snprintf(mqttTopic_32bytes, (sizeof(mqttTopic) - 1), "%s", CONFIG_MQTT_TOPIC);
    snprintf(mqttTopic, sizeof(mqttTopic), "%s/%s", mqttTopic_32bytes, MAC_address);

    const esp_mqtt_client_config_t mqtt_Config = {
        .broker = {
            .address = {
                //.uri = CONFIG_BROKER_URI,
                .hostname = CONFIG_BROKER_HOST,
                .transport = MQTT_TRANSPORT_OVER_TCP,
                //.path = CONFIG_BROKER_URI,
                .port = CONFIG_BROKER_PORT,
            },
        },
        .credentials = {
            .username = CONFIG_MQTT_USERNAME,
            .client_id = CONFIG_MQTT_CLIENT_ID,
            .set_null_client_id = false,
            .authentication = {
                .password = CONFIG_MQTT_PASSWORD,
                // .certificate = (const char *)NULL,
            },
        },
        .network = {
            .disable_auto_reconnect = false,
            .timeout_ms = CONFIG_MQTT_TIMEOUT,
        },
    };

    mqttClient_handle = esp_mqtt_client_init(&mqtt_Config);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqttClient_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, mqttClient_handle);
    esp_mqtt_client_start(mqttClient_handle);

    xTaskCreate(mqttPublishMessage_task, "MQTT Publish", (1024 * 16), NULL, (UBaseType_t)10, &mqttPublishMessageTask_handle);
}

/**
 * @brief SNTP Get time task : init sntp, then get time from ntp and save time to DS3231,
 *        finally delete itself (no loop task)
 *
 * @param parameter
 */
void sntpGetTime_task(void *parameter)
{
    time_t timeNow = 0;
    struct tm timeInfo = {0};
    char current_date_time[64];
    do
    {
        sntp_init_func();
        ESP_ERROR_CHECK_WITHOUT_ABORT(sntp_setTime(&timeInfo, &timeNow));
        mktime(&timeInfo);
        if (timeInfo.tm_year < 130 && timeInfo.tm_year > 120)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_setTime(&ds3231_device, &timeInfo));
            strftime(current_date_time, sizeof(current_date_time), "%H:%M:%S %d/%m/%Y %A", &timeInfo);
            ESP_LOGI(TAG, "Current time in Viet Nam:  %s", current_date_time);
            xEventGroupSetBits(fileStore_eventGroup, FILE_RENAME_FROMSYNC);
        }
        uint64_t seconds_until_new_Day = ds3231_untilNewDay(&ds3231_device);
        if (OTA_Timer != NULL)
        {
            gptimer_alarm_config_t OTA_alarm_config = {
                .alarm_count = seconds_until_new_Day * 1000,
                .flags.auto_reload_on_alarm = false,
            };
            ESP_ERROR_CHECK(gptimer_set_alarm_action(OTA_Timer, &OTA_alarm_config));
            ESP_ERROR_CHECK(gptimer_start(OTA_Timer));
        }
        vTaskDelete(NULL);
    } while (0);
}

/**
 * @brief  This function initialize SNTP, then get time from ntp and save time to DS3231
 *
 *
 */
static void sntp_app_start()
{
    xTaskCreate(sntpGetTime_task, "SNTP Get Time", (1024 * 4), NULL, (UBaseType_t)10, &sntpGetTimeTask_handle);
}
/*          -------------- *** --------------           */
static void initialize_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(error);
}

void getDataFromSensor_task(void *parameters)
{
    struct dataSensor_st dataSensorTemp;
    struct moduleError_st moduleErrorTemp;
    TickType_t task_lastWakeTime;
    task_lastWakeTime = xTaskGetTickCount();
    char readtime[40];
    getDataSensor_semaphore = xSemaphoreCreateMutex();

    for (;;)
    {
#if (CONFIG_USING_RTC)
        if (xSemaphoreTake(getDataSensor_semaphore, portMAX_DELAY))
        {
            moduleErrorTemp.ds3231Error = ds3231_getEpochTime(&ds3231_device, &(dataSensorTemp.timeStamp));
            if (ds3231_isNewDay(&ds3231_device))
            {
                xEventGroupSetBits(fileStore_eventGroup, FILE_RENAME_NEWDAY);
            }
#endif

#if (CONFIG_USING_PMS7003)
            moduleErrorTemp.pms7003Error = pms7003_readData(indoor, &(dataSensorTemp.pm1_0),
                                                            &(dataSensorTemp.pm2_5),
                                                            &(dataSensorTemp.pm10));
#endif

#if (CONFIG_USING_CO2_SENSOR)
            moduleErrorTemp.mhz14aError = mhz14a_getDataFromSensorViaUART(&(dataSensorTemp.CO2));
#endif

#if (CONFIG_USING_BME280)
            moduleErrorTemp.bme280Error = bme280_readSensorData(&bme280_device, &(dataSensorTemp.temperature),
                                                                &(dataSensorTemp.pressure),
                                                                &(dataSensorTemp.humidity));
#endif
            xSemaphoreGive(getDataSensor_semaphore); // Give mutex
            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.ds3231Error);
            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.bme280Error);
            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.pms7003Error);
            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.mhz14aError);

            ESP_LOGI(__func__, "Read data from sensors completed!");

            if (xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdPASS)
            {
                if (xQueueSendToBack(dataSensor_queue, (void *)&dataSensorTemp, WAIT_10_TICK * 5) != pdPASS)
                {

                    ESP_LOGE(__func__, "Failed to post the data sensor to dataSensor Queue.");
                }
                else
                {
                    ds3231_convertTimeToString(&ds3231_device, readtime, 40, 1);
                    ESP_LOGI(__func__, "Success to post the data sensor to dataSensor Queue.");
                }
            }
            xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);

            if (xSemaphoreTake(debug_semaphore, portMAX_DELAY) == pdPASS)
            {
                moduleErrorTemp.timestamp = dataSensorTemp.timeStamp;
                if (xQueueSendToBack(moduleError_queue, (void *)&moduleErrorTemp, WAIT_10_TICK * 3) != pdPASS)
                {
                    ESP_LOGE(__func__, "Failed to post the moduleError to Queue.");
                }
                else
                {
                    ESP_LOGI(__func__, "Success to post the moduleError to Queue.");
                }
            }
            xSemaphoreGive(debug_semaphore);
        }
        memset(&dataSensorTemp, 0, sizeof(struct dataSensor_st));
        memset(&moduleErrorTemp, 0, sizeof(struct moduleError_st));
        vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSOR);
    }
};

/**
 * @brief This task receive data from immediate queue and provide data to MQTT queue and SD queue
 *
 * @param parameters
 */
void allocateDataToMQTTandSDQueue_task(void *parameters)
{
    struct dataSensor_st dataSensorReceiveFromQueue;

    for (;;)
    {
        if (uxQueueMessagesWaiting(dataSensor_queue) != 0)
        {
            if (xQueueReceive(dataSensor_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) == pdPASS)
            {
                ESP_LOGI(__func__, "Receiving data from intermediate queue successfully.");

                if (xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdTRUE)
                {
                    if (xQueueSendToBack(dataSensorSentToSD_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) != pdPASS)
                    {
                        ESP_LOGE(__func__, "Failed to post the data sensor to dataSensorSentToSD Queue.");
                    }
                    else
                    {
                        ESP_LOGI(__func__, "Success to post the data sensor to dataSensorSentToSD Queue.");
                    }

                    if (sendToMQTTQueue == true)
                    {
                        if (xQueueSendToBack(dataSensorSentToMQTT_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(__func__, "Failed to post the data sensor to dataSensorSentToMQTT Queue.");
                        }
                        else
                        {
                            ESP_LOGI(__func__, "Success to post the data sensor to dataSensorSentToMQTT Queue.");
                        }
                    }
#if (CONFIG_USING_LCD_TFT)
                    if (is_LCD_turned_off == false)
                    {
                        if (xQueueSendToBack(lcdDisplay_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(__func__, "Failed to post the data sensor to LCD Display Queue.");
                        }
                        else
                        {
                            ESP_LOGI(__func__, "Success to post the data sensor to LCD Display Queue.");
                        }
                    }
#endif // CONFIG_USING_LCD_TF
                }

                xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);
            }
            else
            {
                ESP_LOGI(__func__, "Receiving data from queue failed.");
                continue;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief This task receive data from module error queue and show device's status by controlling the leds in the curcuit (The Leds were predefined)
 * @param parameters
 */

void DebugLed_task(void *parameters)
{
    struct moduleError_st moduleError;
    uint8_t previous_Debug_Led_status;
    for (;;)
    {
        if (uxQueueMessagesWaiting(moduleError_queue) != 0)
        {
            if (xSemaphoreTake(debug_semaphore, portMAX_DELAY) == pdTRUE)
            {
                if (xQueueReceive(moduleError_queue, (void *)&moduleError, portMAX_DELAY) == pdPASS)
                {
                    ESP_LOGI(__func__, "Receiving data from Module Error queue successfully.");
                    previous_Debug_Led_status = Debug_Led_status;
                    if (moduleError.bme280Error != ESP_OK)
                    {
                        if ((Debug_Led_status & (1 << 4)) == 0x10)
                        {
                            Debug_Led_status &= 0xEF;
                        }
                    }
                    else
                    {
                        if ((Debug_Led_status & (1 << 4)) == 0)
                        {
                            Debug_Led_status |= 0x10;
                        }
                    }

                    if (moduleError.pms7003Error != ESP_OK)
                    {
                        if ((Debug_Led_status & (1 << 5)) == 0x20)
                        {
                            Debug_Led_status &= 0xDF;
                        }
                    }
                    else
                    {
                        if ((Debug_Led_status & (1 << 5)) == 0)
                        {
                            Debug_Led_status |= 0x20;
                        }
                    }

                    if (moduleError.mhz14aError != ESP_OK)
                    {
                        if ((Debug_Led_status & (1 << 6)) == 0x40)
                        {
                            Debug_Led_status &= 0xBF;
                        }
                    }
                    else
                    {
                        if ((Debug_Led_status & (1 << 6)) == 0)
                        {
                            Debug_Led_status |= 0x40;
                        }
                    }
                    if (previous_Debug_Led_status != Debug_Led_status)
                    {
                        pcf8574_port_write(&pcf8574_device, Debug_Led_status); // Write Debug status to PCF8574 I2C port
                    }
                }
                else
                {
                    ESP_LOGE(__func__, "Receiving data from Module Error queue unsuccessfully.");
                }
            }
            xSemaphoreGive(debug_semaphore);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief This task is responsible for showing sensor's data as well as the time that the data was read to the LCD TFT screen
 *
 * @param parameters
 */
void LCDDisplay_task(void *parameters)
{
    struct dataSensor_st displaydata;

    for (;;)
    {
        if (is_LCD_turned_off == false)
        {
            if (ds3231_isNewMinute(&ds3231_device))
            {
                char *datetime_to_display = (char *)malloc(20 * sizeof(char));
                ds3231_convertTimeToString(&ds3231_device, datetime_to_display, 20, 3);
                airsense_date_time_display(datetime_to_display);
                free(datetime_to_display);
            }
            if (uxQueueMessagesWaiting(lcdDisplay_queue) != 0)
            {
                if (xQueueReceive(lcdDisplay_queue, (void *)&displaydata, portMAX_DELAY) == pdPASS)
                {

                    airsense_sensor_data_display(displaydata);
                }
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGE(__func__, "The LCD is turned off");
            vTaskSuspend(NULL);
        }
    }
}

/**
 * @brief This task is responsible for naming SD file
 *
 * @param parameters
 */
void fileEvent_task(void *parameters)
{
    fileStore_eventGroup = xEventGroupCreate();
    SemaphoreHandle_t file_semaphore = NULL;
    file_semaphore = xSemaphoreCreateMutex();
    ds3231_convertTimeToDayString(&ds3231_device, CurrentFileNametoSaveData, 10);

    for (;;)
    {
        EventBits_t bits = xEventGroupWaitBits(fileStore_eventGroup,
                                               FILE_RENAME_NEWDAY | MQTT_CLIENT_CONNECTED | MQTT_CLIENT_DISCONNECTED | FILE_RENAME_FROMSYNC,
                                               pdTRUE,
                                               pdFALSE,
                                               portMAX_DELAY);

        if (xSemaphoreTake(file_semaphore, portMAX_DELAY) == pdTRUE)
        {
            struct tm timeInfo = {0};
            time_t timeNow = 0;
            time(&timeNow);
            localtime_r(&timeNow, &timeInfo);

            if (bits & FILE_RENAME_NEWDAY)
            {
                ds3231_convertTimeToDayString(&ds3231_device, CurrentFileNametoSaveData, 10);
                if (statusDevice.mqttClient == DISCONNECTED)
                {
                    if (strstr(CurrentFileNametoSaveData, "x") == NULL)
                    {
                        strcat(CurrentFileNametoSaveData, "x");
                    }
                    if (xQueueSendToBack(dateTimeLostWiFi_queue, (void *)&timeInfo, WAIT_10_TICK * 5) != pdPASS)
                    {
                        ESP_LOGE(__func__, "Failed to post the data sensor to dateTimeLostWiFi_queue Queue.");
                    }
                    else
                    {
                        ESP_LOGI(__func__, "Success to post the data sensor to dateTimeLostWiFi_queue Queue.");
                    }
                }
            }
            else if (bits & MQTT_CLIENT_CONNECTED)
            {
                if (strstr(CurrentFileNametoSaveData, "x") != NULL)
                {
                    char *position = strrchr(CurrentFileNametoSaveData, 'x');
                    if (position != NULL)
                    {
                        *position = '\0'; // Truncate the string at the underscore position
                    }
                    ESP_LOGI(__func__, "Update file name after reconnect: %s", CurrentFileNametoSaveData);
                }
            }
            else if (bits & MQTT_CLIENT_DISCONNECTED)
            {
                if (strstr(CurrentFileNametoSaveData, "x") == NULL)
                {
                    strcat(CurrentFileNametoSaveData, "x");
                    ESP_LOGI(__func__, "Update file name after disconnect: %s", CurrentFileNametoSaveData);
                }
                if (xQueueSendToBack(dateTimeLostWiFi_queue, (void *)&timeInfo, WAIT_10_TICK * 5) != pdPASS)
                {
                    ESP_LOGE(__func__, "Failed to post the data sensor to dateTimeLostWiFi_queue Queue.");
                }
                else
                {
                    ESP_LOGI(__func__, "Success to post the data sensor to dateTimeLostWiFi_queue Queue.");
                }
                printf("Time: %d-%d-%d\n", timeInfo.tm_mday, timeInfo.tm_mon + 1, timeInfo.tm_year + 1900);
            }
            else if (bits & FILE_RENAME_FROMSYNC)
            {
                ds3231_convertTimeToDayString(&ds3231_device, CurrentFileNametoSaveData, 10);
                ESP_LOGI(__func__, "File name updated from SNTP");
            }
            xSemaphoreGive(file_semaphore);
        }
    }
};

/**
 * @brief This task read data from SD card (saved when lost Wifi) and push data to MQTT and SD queue
 *
 * @param parameters
 */
void sendDataSensorToMQTTServerAfterReconnectWiFi_task(void *parameters)
{
    struct tm timeLostWiFi = {0};
    char CurrentFileNametoSaveDataLostWiFi[21];

    for (;;)
    {
        while (uxQueueMessagesWaiting((const QueueHandle_t)dateTimeLostWiFi_queue) != 0)
        {
            memset(&timeLostWiFi, 0, sizeof(struct tm));
            memset(CurrentFileNametoSaveDataLostWiFi, 0, sizeof(CurrentFileNametoSaveDataLostWiFi));

            if (xQueueReceive(dateTimeLostWiFi_queue, (void *)&timeLostWiFi, portMAX_DELAY))
            {
                sprintf(CurrentFileNametoSaveDataLostWiFi, "%" PRIu8 "%" PRIu8 "%" PRIu32 "x", (uint8_t)timeLostWiFi.tm_mday, (uint8_t)(timeLostWiFi.tm_mon + 1), (uint32_t)(timeLostWiFi.tm_year) + 1900);
            }
            char pathFile[64];
            sprintf(pathFile, "%s/%s.txt", mount_point, CurrentFileNametoSaveDataLostWiFi);
            char *dataSensorString;
            struct dataSensor_st dataSensorTemp = {0};
            dataSensorString = (char *)malloc((size_t)256);
            ESP_LOGI(__func__, "Opening file %s...", pathFile);
            FILE *file = fopen(pathFile, "r");
            if (file == NULL)
            {
                ESP_LOGE(__func__, "Failed to open file for reading.");
                continue;
            }

            while (!feof(file))
            {
                memset(dataSensorString, 0, strlen(dataSensorString));
                memset(&dataSensorTemp, 0, sizeof(struct dataSensor_st));
                fscanf(file, "%[^,],%llu,%f,%f,%f,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", dataSensorString,
                       &(dataSensorTemp.timeStamp),
                       &(dataSensorTemp.temperature),
                       &(dataSensorTemp.humidity),
                       &(dataSensorTemp.pressure),
                       &(dataSensorTemp.pm1_0),
                       &(dataSensorTemp.pm2_5),
                       &(dataSensorTemp.pm10),
                       &(dataSensorTemp.CO2));

                /**
                 * @warning: Do not use xSemaphoreTake() function here!!!
                 */
                // while (!(uxQueueMessagesWaiting(dataSensorSentToMQTT_queue) < (QUEUE_SIZE - 1)));

                BaseType_t flag_checkSendToQueueSuccess = pdFAIL;

                do
                {
                    if (xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdPASS)
                    {
                        flag_checkSendToQueueSuccess = xQueueSendToBack(dataSensor_queue, (void *)&dataSensorTemp, WAIT_100_TICK);
                        if (flag_checkSendToQueueSuccess != pdPASS)
                        {
                            ESP_LOGW(__func__, "Failed to post the data sensor to dataSensorIntermediate Queue.  Queue size: %d (max 10).", uxQueueMessagesWaiting(dataSensorSentToMQTT_queue));
                        }
                        else
                        {
                            ESP_LOGI(__func__, "Success to post the data sensor to dataSensorIntermediate Queue.");
                        }
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                    xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);
                } while (flag_checkSendToQueueSuccess != pdPASS);

                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            free(dataSensorString);

            // After read all data from "...x" file in SD card, delete that file
            ESP_ERROR_CHECK_WITHOUT_ABORT(sdcard_removeFile(CurrentFileNametoSaveDataLostWiFi));
        }
        // Suspend ourselves.
        vTaskSuspend(NULL);
    }
}

/**
 * @brief Save data from SD queue to SD card
 *
 * @param parameters
 */
void saveDataSensorToSDcard_task(void *parameters)
{
    struct dataSensor_st dataSensorReceiveFromQueue;
    writeDataToSDcard_semaphore = xSemaphoreCreateMutex();
    bool is_sd_card_OK = true;
    bool is_sd_card_previous_OK = true;
    for (;;)
    {
        if (uxQueueMessagesWaiting(dataSensorSentToSD_queue) != 0) // Check if dataSensorSentToSD_queue is empty
        {
            if (xQueueReceive(dataSensorSentToSD_queue, (void *)&dataSensorReceiveFromQueue, WAIT_10_TICK * 50) == pdPASS) // Get data sesor from queue
            {
                ESP_LOGI(__func__, "Receiving data from queue successfully.");

                if (xSemaphoreTake(writeDataToSDcard_semaphore, portMAX_DELAY) == pdTRUE)
                {
                    static esp_err_t errorCode_t;
                    // Create data string follow format
                    errorCode_t = sdcard_writeDataToFile(CurrentFileNametoSaveData, "%s,%llu,%.2f,%.2f,%.2f,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "\n", CONFIG_NAME_DEVICE,
                                                         dataSensorReceiveFromQueue.timeStamp,
                                                         dataSensorReceiveFromQueue.temperature,
                                                         dataSensorReceiveFromQueue.humidity,
                                                         dataSensorReceiveFromQueue.pressure,
                                                         dataSensorReceiveFromQueue.pm1_0,
                                                         dataSensorReceiveFromQueue.pm2_5,
                                                         dataSensorReceiveFromQueue.pm10,
                                                         dataSensorReceiveFromQueue.CO2);
                    ESP_LOGI(TAG, "Save task received mutex!");
                    xSemaphoreGive(writeDataToSDcard_semaphore);
                    if (errorCode_t != ESP_OK)
                    {
                        ESP_LOGE(__func__, "sdcard_writeDataToFile(...) function returned error: 0x%.4X", errorCode_t);
#if (CONFIG_USING_LCD_TFT)
                        is_sd_card_OK = false;
                        if ((is_LCD_turned_off == false) && (is_sd_card_OK != is_sd_card_previous_OK))
                        {
                            airsense_sdcard_state_display(is_sd_card_OK);
                        }
                        is_sd_card_previous_OK = is_sd_card_OK;
#endif
                    }
#if (CONFIG_USING_LCD_TFT)
                    else
                    {
                        is_sd_card_OK = true;
                        if ((is_LCD_turned_off == false) && (is_sd_card_OK != is_sd_card_previous_OK))
                        {
                            airsense_sdcard_state_display(is_sd_card_OK);
                        }
                        is_sd_card_previous_OK = is_sd_card_OK;
                    }
#endif
                }
            }
            else
            {
                ESP_LOGI(__func__, "Receiving data from queue failed.");
                continue;
            }
        }

        vTaskDelay(PERIOD_SAVE_DATA_SENSOR_TO_SDCARD);
    }
};

/*------------------------------------ MAIN_APP ------------------------------------*/

void app_main(void)
{

    // Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(__func__, "Starting app main.");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK)
    {
        printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
               (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    }
    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    ESP_LOGI(__func__, "Name device: %s.", CONFIG_NAME_DEVICE);
    ESP_LOGI(__func__, "Firmware version %s.", CONFIG_FIRMWARE_VERSION);

    // Initialize nvs partition
    ESP_LOGI(__func__, "Initialize nvs partition.");
    initialize_nvs();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

    // Wait a second for memory initialization
    vTaskDelay(500 / portTICK_PERIOD_MS);

#if (CONFIG_USING_FIRMWARE_UPDATE_BY_OTA)
    // Register eventhandler for OTA
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &OTA_event_handler, NULL));

    // Initialize timer for OTA
    gptimer_config_t OTA_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick= 1/resolution = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&OTA_timer_config, &OTA_Timer));

    // Resgister callback for timer OTA
    gptimer_event_callbacks_t OTA_cbs = {
        .on_alarm = OTA_timer_isr_handler,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(OTA_Timer, &OTA_cbs, NULL));
    ESP_LOGI(TAG, "Enable OTA timer");
    ESP_ERROR_CHECK(gptimer_enable(OTA_Timer));

    xOTA_reset_timer_Semaphore = xSemaphoreCreateBinary();
    xOTASemaphore = xSemaphoreCreateBinary();
#endif // CONFIG_USING_FIRMWARE_UPDATE_BY_OTA
    // Initialize I2C Driver

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    // Debug Leds and DS3231 use same I2C port
    // Initialize DEBUG LED
    ESP_ERROR_CHECK_WITHOUT_ABORT(pcf8574_init_desc(&pcf8574_device, PCF8574_ADDR, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
    pcf8574_port_write(&pcf8574_device, Debug_Led_status);

// Initialize SD card
#if (CONFIG_USING_SDCARD)
    // Initialize SPI Bus

    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t_1 = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    esp_err_t sd_err = sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t_1, &slot_config);
    ESP_ERROR_CHECK_WITHOUT_ABORT(sd_err);
    if (sd_err != ESP_OK)
    {
        Debug_Led_status &= 0xFB;
        pcf8574_port_write(&pcf8574_device, Debug_Led_status);
    }
#endif // CONFIG_USING_SDCARD

// Initialize BME280 Sensor
#if (CONFIG_USING_BME280)

    ESP_LOGI(__func__, "Initialize BME280 sensor(I2C/Wire%d).", CONFIG_BME_I2C_PORT);

    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(&bme280_device, &bme280_params, BME280_ADDRESS,
                                              CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL));

#endif // CONFIG_USING_BME280

// Initialize RTC module
#if (CONFIG_USING_RTC)
    ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);

    memset(&ds3231_device, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_initialize(&ds3231_device, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
#endif // CONFIG_USING_RTC

#if (CONFIG_USING_CO2_SENSOR)
    uint32_t *co2_t = malloc(sizeof(uint32_t));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mhz14a_initUART(&mhz14a_uart_config));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mhz14a_setRangeSetting(co2_range0To5000));
    while (mhz14a_getDataFromSensorViaUART(&co2_t) != ESP_OK)
        ;

#endif // CONFIG_USING_CO2_SENSOR

#if (CONFIG_USING_PMS7003)
    ESP_ERROR_CHECK_WITHOUT_ABORT(pms7003_initUart(&pms_uart_config));

    uint32_t pm1p0_t, pm2p5_t, pm10_t;
    while (pms7003_readData(indoor, &pm1p0_t, &pm2p5_t, &pm10_t) != ESP_OK)
        ; // Waiting for PMS7003 sensor read data from RX buffer

#endif // CONFIG_USING_PMS7003

#if (CONFIG_USING_LCD_TFT)
    // Define a button for turning on or off the LCD sreen
    // Pointer to LCD's main display
    gpio_config_t on_off_lcd_button =
        {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = 1,
            .pull_down_en = 0,
            .pin_bit_mask = (1ULL << SW_1)};
    gpio_config(&on_off_lcd_button);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SW_BUILT_IN, on_off_lcd_handler, (void *)SW_BUILT_IN);

    ili9341_init(&lcd_mainscreen);
    ui_airsense_init(lcd_mainscreen);

#if (CONFIG_USING_SDCARD)
    if (sd_err == ESP_OK)
    {
        airsense_sdcard_state_display(true);
    }
#endif
#endif // CONFIG_USING_LCD_TFT

    xTaskCreate(fileEvent_task, "EventFile",
                (1024 * 8),
                NULL,
                (UBaseType_t)7,
                NULL);

    // Creat dataSensorIntermediateQueue
    dataSensor_queue = xQueueCreate(30, sizeof(struct dataSensor_st));
    while (dataSensor_queue == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorIntermediate Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorIntermediate Queue...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        dataSensor_queue = xQueueCreate(30, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create dataSensorIntermediate Queue success.");

    // Create dataSensorQueue
    dataSensorSentToSD_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    while (dataSensorSentToSD_queue == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorSentToSD Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorSentToSD Queue...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        dataSensorSentToSD_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create dataSensorSentToSD Queue success.");

    // Create moduleErrorQueue
    moduleError_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct moduleError_st));
    for (size_t i = 0; moduleError_queue == NULL && i < 5; i++)
    {
        ESP_LOGE(__func__, "Create moduleError Queue failed.");
        ESP_LOGI(__func__, "Retry to create moduleError Queue...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        moduleError_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct moduleError_st));
    }

    if (moduleError_queue == NULL)
    {
        ESP_LOGE(__func__, "Create moduleError Queue failed.");
        ESP_LOGE(__func__, "ModuleErrorQueue created fail. All errors during getDataFromSensor_task() function running will not write to SD card!");
    }
    else
    {
        ESP_LOGI(__func__, "Create moduleError Queue success.");
    }

    // Create dataSensorSentToMQTT Queue
    dataSensorSentToMQTT_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    while (dataSensorSentToMQTT_queue == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorSentToMQTT Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorSentToMQTT Queue...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        dataSensorSentToMQTT_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create dataSensorSentToMQTT Queue success.");
    // Create LCD Display Queue (queue size = 1)
#if (CONFIG_USING_LCD_TFT)
    lcdDisplay_queue = xQueueCreate(1, sizeof(struct dataSensor_st));
    while (lcdDisplay_queue == NULL)
    {
        ESP_LOGE(__func__, "Create LCD Display Queue unsuccessfully.");
        ESP_LOGI(__func__, "Retry to create LCD Display Queue...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        lcdDisplay_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create LCD Display Queue successfully.");
#endif
    // Create dateTimeLostWiFi_queue Queue
    dateTimeLostWiFi_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct tm));
    if (dateTimeLostWiFi_queue == NULL)
    {
        ESP_LOGE("Booting", "Create CurrentFileNametoSaveData Queue failed.");
    }
    ESP_LOGI(__func__, "Create dateTimeLostWiFi Queue successfully.");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Create task to get data from sensor (64Kb stack memory| priority 25(max))
    // Period 5000ms
    xTaskCreate(getDataFromSensor_task, "GetDataSensor", (1024 * 32), NULL, (UBaseType_t)25, &getDataFromSensorTask_handle);

    // Create task to save data from sensor read by getDataFromSensor_task() to SD card (16Kb stack memory| priority 10)
    // Period 5000ms
    xTaskCreate(saveDataSensorToSDcard_task, "SaveDataSensor", (1024 * 16), NULL, (UBaseType_t)10, &saveDataSensorToSDcardTask_handle);

#if (CONFIG_USING_LCD_TFT)
    // Create task to display sensor data to LCD TFT screen (16Kb stack memory| priority 16)
    // Period 5000ms
    xTaskCreate(LCDDisplay_task, "LCD Display task", (1024 * 16), NULL, (UBaseType_t)16, &display_lcd_tft_handle);
#endif // CONFIG_USING_LCD_TFT

    // Create task to show status or notify about sensor or SD card errors (6Kb stack memory| priority 16)
    // Period 5000ms
    xTaskCreate(DebugLed_task, "DebugTask", (1024 * 6), NULL, (UBaseType_t)16, NULL);

    // Semaphore that handle the Module Error queue
    debug_semaphore = xSemaphoreCreateMutex();

    // Semaphore that handle the immediate queue (that be used by 3 tasks)
    allocateDataToMQTTandSDQueue_semaphore = xSemaphoreCreateMutex();

    // Creat task to allocate data to MQTT and SD queue from dataSensorIntermediate queue
    xTaskCreate(allocateDataToMQTTandSDQueue_task, "AllocateData", (1024 * 16), NULL, (UBaseType_t)15, &allocateDataToMQTTandSDQueue_handle);
    if (allocateDataToMQTTandSDQueue_handle != NULL)
    {
        ESP_LOGI(__func__, "Create task AllocateData successfully.");
    }
    else
    {
        ESP_LOGE(__func__, "Create task AllocateData failed.");
    }
#if (CONFIG_USING_WIFI)
    WIFI_initSTA();
#endif
}
