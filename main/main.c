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
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_spi_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "bme280.h"
#include "sdcard.h"
#include "pms7003.h"
#include "DS3231Time.h"
#include "datamanager.h"
#include "DeviceManager.h"
#include "sntp_sync.h"

/*------------------------------------ DEFINE ------------------------------------ */

__attribute__((unused)) static const char *TAG = "Main";

#define PERIOD_GET_DATA_FROM_SENSOR (TickType_t)(5000 / portTICK_RATE_MS)
#define PERIOD_SAVE_DATA_SENSOR_TO_SDCARD (TickType_t)(2500 / portTICK_RATE_MS)
#define PERIOD_SAVE_DATA_AFTER_WIFI_RECONNECT (TickType_t)(1000 / portTICK_RATE_MS)

#define NO_WAIT (TickType_t)(0)
#define WAIT_10_TICK (TickType_t)(10 / portTICK_RATE_MS)
#define WAIT_100_TICK (TickType_t)(100 / portTICK_RATE_MS)

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

esp_mqtt_client_handle_t mqttClient_handle = NULL;

TaskHandle_t getDataFromSensorTask_handle = NULL;
TaskHandle_t saveDataSensorToSDcardTask_handle = NULL;
TaskHandle_t saveDataSensorAfterReconnectWiFiTask_handle = NULL;
TaskHandle_t mqttPublishMessageTask_handle = NULL;
TaskHandle_t sntpGetTimeTask_handle = NULL;
TaskHandle_t allocateDataToMQTTandSDQueue_handle = NULL;

SemaphoreHandle_t getDataSensor_semaphore = NULL;
SemaphoreHandle_t writeDataToSDcard_semaphore = NULL;
SemaphoreHandle_t sentDataToMQTT_semaphore = NULL;
SemaphoreHandle_t writeDataToSDcardNoWifi_semaphore = NULL;
SemaphoreHandle_t allocateDataToMQTTandSDQueue_semaphore = NULL;

QueueHandle_t dataSensorSentToSD_queue;
QueueHandle_t dataSensorSentToMQTT_queue;
QueueHandle_t moduleError_queue;
QueueHandle_t nameFileSaveDataNoWiFi_queue;
QueueHandle_t dateTimeLostWiFi_queue;
QueueHandle_t dataSensorIntermediate_queue;

static struct statusDevice_st statusDevice = {0};

static char nameFileSaveData[21];
char mqttTopic[32];
uint8_t MAC_address[6];

// Whether send data to MQTT queue or not (depend on WIFI connection)
bool sendToMQTTQueue = false;

const char *formatDataSensorString = "{\n\t\"station_id\":\"%x%x%x%x\",\n\t\"Time\":%lld,\n\t\"Temperature\":%.2f,\n\t\"Humidity\":%.2f,\n\t\"Pressure\":%.2f,\n\t\"PM1\":%d,\n\t\"PM2p5\":%d,\n\t\"PM10\":%d\n}";

//------------------------------------------------------------------

i2c_dev_t ds3231_device;

bmp280_t bme280_device;
bmp280_params_t bme280_params;

uart_config_t pms_uart_config = UART_CONFIG_DEFAULT();

/*------------------------------------ WIFI ------------------------------------ */

static void mqtt_app_start(void);
static void sntp_app_start(void);
void sendDataSensorToMQTTServerAfterReconnectWiFi_task(void *parameters);

static esp_err_t WiFi_eventHandler(void *argument, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(__func__, "Trying to connect with Wi-Fi\n");
        break;

    case SYSTEM_EVENT_STA_CONNECTED:
        ESP_LOGI(__func__, "Wi-Fi connected AP SSID:%s password:%s\n", CONFIG_SSID, CONFIG_PASSWORD);
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(__func__, "got ip: starting MQTT Client\n");
#ifdef CONFIG_RTC_TIME_SYNC
        if (sntpGetTimeTask_handle == NULL)
        {
            sntp_app_start();
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
                xTaskCreate(sendDataSensorToMQTTServerAfterReconnectWiFi_task, "SendDataAfterConnect", (1024 * 16), NULL, (UBaseType_t)15, &saveDataSensorAfterReconnectWiFiTask_handle);
                if(saveDataSensorAfterReconnectWiFiTask_handle != NULL)
                {
                    ESP_LOGW(__func__, "Create task reconnected OK.");
                } else {
                    ESP_LOGE(__func__, "Create task reconnected failed.");
                }
            } else if (eTaskGetState(saveDataSensorAfterReconnectWiFiTask_handle) == eSuspended) {
                vTaskResume(saveDataSensorAfterReconnectWiFiTask_handle);
                ESP_LOGI(__func__, "Resume task saveDataSensorAfterReconnectWiFi.");
            }
        }
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* When esp32 disconnect to wifi, this event become
         * active. We suspend the task mqttPublishMessageTask. */
        ESP_LOGI(__func__, "disconnected: Retrying Wi-Fi connect to AP SSID:%s password:%s", CONFIG_SSID, CONFIG_PASSWORD);
        if (mqttPublishMessageTask_handle != NULL && eTaskGetState(mqttPublishMessageTask_handle) != eSuspended)
        {
            vTaskSuspend(mqttPublishMessageTask_handle);
            statusDevice.mqttClient = DISCONNECTED;
            xEventGroupSetBits(fileStore_eventGroup, MQTT_CLIENT_DISCONNECTED);
            sendToMQTTQueue = false;
            ESP_LOGI(__func__, "set bit disconnect");
        }
        esp_wifi_connect();
        break;

    default:
        break;
    }
    return ESP_OK;
}

/**
 * @brief This function initialize wifi and create, start WiFi handle such as loop (low priority)
 *
 */
void WIFI_initSTA(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t WIFI_initConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_init(&WIFI_initConfig));

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_init(WiFi_eventHandler, NULL));

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
    ESP_LOGD(__func__, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(__func__, "MQTT_EVENT_CONNECTED");
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
                        esp_err_t error = 0;
                        WORD_ALIGNED_ATTR char mqttMessage[256];
                        sprintf(mqttMessage, formatDataSensorString, MAC_address[0],
                                MAC_address[1],
                                MAC_address[2],
                                MAC_address[3],
                                dataSensorReceiveFromQueue.timeStamp,
                                dataSensorReceiveFromQueue.temperature,
                                dataSensorReceiveFromQueue.humidity,
                                dataSensorReceiveFromQueue.pressure,
                                dataSensorReceiveFromQueue.pm1_0,
                                dataSensorReceiveFromQueue.pm2_5,
                                dataSensorReceiveFromQueue.pm10);

                        error = esp_mqtt_client_publish(mqttClient_handle, (const char *)mqttTopic, mqttMessage, 0, 0, 0);
                        xSemaphoreGive(sentDataToMQTT_semaphore);
                        if (error == ESP_FAIL)
                        {
                            ESP_LOGE(__func__, "MQTT client publish message failed ¯\\_(ツ)_/¯...");
                        }
                        else
                        {
                            ESP_LOGI(__func__, "MQTT client publish message success (^人^).");
                        }
                    }
                    vTaskDelay((TickType_t)(1000 / portTICK_RATE_MS));
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
    const esp_mqtt_client_config_t mqtt_Config = {
        .host = CONFIG_BROKER_HOST,
        .uri = CONFIG_BROKER_URI,
        .port = CONFIG_BROKER_PORT,
        .username = CONFIG_MQTT_USERNAME,
        .password = CONFIG_MQTT_PASSWORD,
        //.cert_pem = (const char *)"",
    };

    ESP_LOGI(__func__, "Free memory: %d bytes", esp_get_free_heap_size());
    mqttClient_handle = esp_mqtt_client_init(&mqtt_Config);

    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqttClient_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, mqttClient_handle);
    esp_mqtt_client_start(mqttClient_handle);
    esp_read_mac(MAC_address, ESP_MAC_WIFI_STA); // Get MAC address of ESP32
    sprintf(mqttTopic, "%s/%x:%x:%x:%x:%x:%x", "IDF", MAC_address[0], MAC_address[1], MAC_address[2], MAC_address[3], MAC_address[4], MAC_address[5]);

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
    do
    {
        sntp_init_func();
        ESP_ERROR_CHECK_WITHOUT_ABORT(sntp_setTime(&timeInfo, &timeNow));
        mktime(&timeInfo);
        if (timeInfo.tm_year < 130 && timeInfo.tm_year > 120)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_setTime(&ds3231_device, &timeInfo));
            xEventGroupSetBits(fileStore_eventGroup, FILE_RENAME_FROMSYNC);
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

#if (CONFIG_USING_BME280)
            moduleErrorTemp.bme280Error = bme280_readSensorData(&bme280_device, &(dataSensorTemp.temperature),
                                                                &(dataSensorTemp.pressure),
                                                                &(dataSensorTemp.humidity));
#endif
            xSemaphoreGive(getDataSensor_semaphore); // Give mutex

            printf("%s,%llu,%.2f,%.2f,%.2f,%d,%d,%d\n", CONFIG_NAME_DEVICE,
                   dataSensorTemp.timeStamp,
                   dataSensorTemp.temperature,
                   dataSensorTemp.humidity,
                   dataSensorTemp.pressure,
                   dataSensorTemp.pm1_0,
                   dataSensorTemp.pm2_5,
                   dataSensorTemp.pm10);

            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.ds3231Error);
            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.bme280Error);
            ESP_ERROR_CHECK_WITHOUT_ABORT(moduleErrorTemp.pms7003Error);

            ESP_LOGI(__func__, "Read data from sensors completed!");

            if (xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdPASS)
            {
                if (xQueueSendToBack(dataSensorIntermediate_queue, (void *)&dataSensorTemp, WAIT_10_TICK * 5) != pdPASS)
                {
                    ESP_LOGE(__func__, "Failed to post the data sensor to dataSensorIntermediate Queue.");
                }
                else
                {
                    ESP_LOGI(__func__, "Success to post the data sensor to dataSensorIntermediate Queue.");
                }
            }
            xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);

            if (moduleError_queue != NULL &&
                (moduleErrorTemp.ds3231Error != ESP_OK ||
                 moduleErrorTemp.bme280Error != ESP_OK ||
                 moduleErrorTemp.pms7003Error != ESP_OK))
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
        if (uxQueueMessagesWaiting(dataSensorIntermediate_queue) != 0)
        {
            if (xQueueReceive(dataSensorIntermediate_queue, (void *)&dataSensorReceiveFromQueue, portMAX_DELAY) == pdPASS)
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
                }
                
                xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);
            }
            else
            {
                ESP_LOGI(__func__, "Receiving data from queue failed.");
                continue;
            }
        }
        vTaskDelay(100 / portTICK_RATE_MS);
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
    ds3231_convertTimeToString(&ds3231_device, nameFileSaveData, 10);

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
                ds3231_convertTimeToString(&ds3231_device, nameFileSaveData, 10);
                if (statusDevice.mqttClient == DISCONNECTED)
                {
                    if (strstr(nameFileSaveData, "_noWiFi") == NULL)
                    {
                        strcat(nameFileSaveData, "_noWiFi");
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
                if (strstr(nameFileSaveData, "_noWiFi") != NULL)
                {
                    char *position = strrchr(nameFileSaveData, '_');
                    if (position != NULL)
                    {
                        *position = '\0'; // Truncate the string at the underscore position
                    }
                    ESP_LOGI(__func__, "Update file name after reconnect: %s", nameFileSaveData);
                }
            }
            else if (bits & MQTT_CLIENT_DISCONNECTED)
            {
                if (strstr(nameFileSaveData, "_noWiFi") == NULL)
                {
                    strcat(nameFileSaveData, "_noWiFi");
                    ESP_LOGI(__func__, "Update file name after disconnect: %s", nameFileSaveData);
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
                ds3231_convertTimeToString(&ds3231_device, nameFileSaveData, 10);
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
    char nameFileSaveDataLostWiFi[21];

    for (;;)
    {
        while (uxQueueMessagesWaiting((const QueueHandle_t)dateTimeLostWiFi_queue) != 0)
        {
            memset(&timeLostWiFi, 0, sizeof(struct tm));
            memset(nameFileSaveDataLostWiFi, 0, sizeof(nameFileSaveDataLostWiFi));

            if (xQueueReceive(dateTimeLostWiFi_queue, (void *)&timeLostWiFi, portMAX_DELAY))
            {
                sprintf(nameFileSaveDataLostWiFi, "%d-%d-%d_noWiFi", (uint8_t)timeLostWiFi.tm_mday, (uint8_t)(timeLostWiFi.tm_mon + 1), (uint8_t)(timeLostWiFi.tm_year) + 1900);
            }
            char pathFile[64];
            sprintf(pathFile, "%s/%s.txt", mount_point, nameFileSaveDataLostWiFi);
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

            // while (*file != EOF)
            while (!feof(file))
            {
                memset(dataSensorString, 0, strlen(dataSensorString));
                memset(&dataSensorTemp, 0, sizeof(struct dataSensor_st));
                fscanf(file, "%[^,],%llu,%f,%f,%f,%u,%u,%u\n", dataSensorString,
                       &(dataSensorTemp.timeStamp),
                       &(dataSensorTemp.temperature),
                       &(dataSensorTemp.humidity),
                       &(dataSensorTemp.pressure),
                       &(dataSensorTemp.pm1_0),
                       &(dataSensorTemp.pm2_5),
                       &(dataSensorTemp.pm10));

                /**
                 * @warning: Do not use xSemaphoreTake() function here!!!
                 */
                // while (!(uxQueueMessagesWaiting(dataSensorSentToMQTT_queue) < (QUEUE_SIZE - 1)));

                BaseType_t flag_checkSendToQueueSuccess = pdFAIL;

                do
                {
                    if(xSemaphoreTake(allocateDataToMQTTandSDQueue_semaphore, portMAX_DELAY) == pdPASS)
                    {
                        flag_checkSendToQueueSuccess = xQueueSendToBack(dataSensorIntermediate_queue, (void *)&dataSensorTemp, WAIT_100_TICK);
                        if (flag_checkSendToQueueSuccess != pdPASS)
                        {
                            ESP_LOGW(__func__, "Failed to post the data sensor to dataSensorIntermediate Queue.  Queue size: %u (max 10).", uxQueueMessagesWaiting(dataSensorSentToMQTT_queue));
                        }
                        else
                        {
                            ESP_LOGI(__func__, "Success to post the data sensor to dataSensorIntermediate Queue.");
                        }
                        vTaskDelay(100 / portTICK_RATE_MS);
                    }
                    xSemaphoreGive(allocateDataToMQTTandSDQueue_semaphore);
                } while (flag_checkSendToQueueSuccess != pdPASS);

                vTaskDelay(100 / portTICK_RATE_MS);
            }
            free(dataSensorString);

            // After read all data from "..._noWifi" file in SD card, delete that file
            ESP_ERROR_CHECK_WITHOUT_ABORT(sdcard_removeFile(nameFileSaveDataLostWiFi));
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
                    errorCode_t = sdcard_writeDataToFile(nameFileSaveData, "%s,%llu,%.2f,%.2f,%.2f,%d,%d,%d\n", CONFIG_NAME_DEVICE,
                                                         dataSensorReceiveFromQueue.timeStamp,
                                                         dataSensorReceiveFromQueue.temperature,
                                                         dataSensorReceiveFromQueue.humidity,
                                                         dataSensorReceiveFromQueue.pressure,
                                                         dataSensorReceiveFromQueue.pm1_0,
                                                         dataSensorReceiveFromQueue.pm2_5,
                                                         dataSensorReceiveFromQueue.pm10);
                    ESP_LOGI(TAG, "Save task received mutex!");
                    xSemaphoreGive(writeDataToSDcard_semaphore);
                    if (errorCode_t != ESP_OK)
                    {
                        ESP_LOGE(__func__, "sdcard_writeDataToFile(...) function returned error: 0x%.4X", errorCode_t);
                    }
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
    // ESP_LOGI(__func__, "Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(__func__, "IDF version: %s", esp_get_idf_version());
    ESP_LOGI("ESP_info", "Minimum free heap size: %d bytes\r\n", esp_get_minimum_free_heap_size());

    // Booting firmware
    ESP_LOGI(__func__, "Booting....");
    ESP_LOGI(__func__, "Name device: %s.", CONFIG_NAME_DEVICE);
    ESP_LOGI(__func__, "Firmware version %s.", CONFIG_FIRMWARE_VERSION);

    // Initialize nvs partition
    ESP_LOGI(__func__, "Initialize nvs partition.");
    initialize_nvs();
    // Wait a second for memory initialization
    vTaskDelay(1000 / portTICK_RATE_MS);

// Smartconfig
// configSmartWifi();

// Initialize SD card
#if (CONFIG_USING_SDCARD)
    // Initialize SPI Bus

    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));
#endif // CONFIG_USING_SDCARD

// Initialize BME280 Sensor
#if (CONFIG_USING_BME280)
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    ESP_LOGI(__func__, "Initialize BME280 sensor(I2C/Wire%d).", CONFIG_BME_I2C_PORT);

    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(&bme280_device, &bme280_params, BME280_ADDRESS,
                                              CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL));

#endif // CONFIG_USING_BME280

// Initialize RTC module
#if (CONFIG_USING_RTC)
    ESP_LOGI(__func__, "Initialize DS3231 module(I2C/Wire%d).", CONFIG_RTC_I2C_PORT);

    memset(&ds3231_device, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK_WITHOUT_ABORT(ds3231_initialize(&ds3231_device, CONFIG_RTC_I2C_PORT, CONFIG_RTC_PIN_NUM_SDA, CONFIG_RTC_PIN_NUM_SCL));
    // ds3231_convertTimeToString(&ds3231_device, nameFileSaveData, 10);
#endif // CONFIG_USING_RTC

#if (CONFIG_USING_PMS7003)
    ESP_ERROR_CHECK_WITHOUT_ABORT(pms7003_initUart(&pms_uart_config));

    uint32_t pm1p0_t, pm2p5_t, pm10_t;
    while (pms7003_readData(indoor, &pm1p0_t, &pm2p5_t, &pm10_t) != ESP_OK); // Waiting for PMS7003 sensor read data from RX buffer

#endif // CONFIG_USING_PMS7003

    xTaskCreate(fileEvent_task, "EventFile",
                (1024 * 8),
                NULL,
                (UBaseType_t)7,
                NULL);

    // Creat dataSensorIntermediateQueue
    dataSensorIntermediate_queue = xQueueCreate(30, sizeof(struct dataSensor_st));
    while (dataSensorIntermediate_queue == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorIntermediate Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorIntermediate Queue...");
        vTaskDelay(500 / portTICK_RATE_MS);
        dataSensorIntermediate_queue = xQueueCreate(30, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create dataSensorIntermediate Queue success.");

    // Create dataSensorQueue
    dataSensorSentToSD_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    while (dataSensorSentToSD_queue == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorSentToSD Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorSentToSD Queue...");
        vTaskDelay(500 / portTICK_RATE_MS);
        dataSensorSentToSD_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create dataSensorSentToSD Queue success.");

    // Create moduleErrorQueue
    moduleError_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct moduleError_st));
    for (size_t i = 0; moduleError_queue == NULL && i < 5; i++)
    {
        ESP_LOGE(__func__, "Create moduleError Queue failed.");
        ESP_LOGI(__func__, "Retry to create moduleError Queue...");
        vTaskDelay(500 / portTICK_RATE_MS);
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
        vTaskDelay(500 / portTICK_RATE_MS);
        dataSensorSentToMQTT_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };
    ESP_LOGI(__func__, "Create dataSensorSentToMQTT Queue success.");

    // Create dateTimeLostWiFi_queue Queue
    dateTimeLostWiFi_queue = xQueueCreate(QUEUE_SIZE, sizeof(struct tm));
    if (dateTimeLostWiFi_queue == NULL)
    {
        ESP_LOGE("Booting", "Create nameFileSaveData Queue failed.");
    }
    ESP_LOGI(__func__, "Create dateTimeLostWiFi Queue success.");

    // Create task to get data from sensor (64Kb stack memory| priority 25(max))
    // Period 5000ms
    xTaskCreate(getDataFromSensor_task, "GetDataSensor", (1024 * 32), NULL, (UBaseType_t)25, &getDataFromSensorTask_handle);

    // Create task to save data from sensor read by getDataFromSensor_task() to SD card (16Kb stack memory| priority 10)
    // Period 5000ms
    xTaskCreate(saveDataSensorToSDcard_task, "SaveDataSensor", (1024 * 16), NULL, (UBaseType_t)10, &saveDataSensorToSDcardTask_handle);

    // Semaphore that handle the immediate queue (that be used by 3 tasks) 
    allocateDataToMQTTandSDQueue_semaphore = xSemaphoreCreateMutex();

    // Creat task to allocate data to MQTT and SD queue from dataSensorIntermediate queue
    xTaskCreate(allocateDataToMQTTandSDQueue_task, "AllocateData", (1024 * 16), NULL, (UBaseType_t)15, &allocateDataToMQTTandSDQueue_handle);
    if (allocateDataToMQTTandSDQueue_handle != NULL)
    {
        ESP_LOGI(__func__, "Create task AllocateData successfully.");
    } else {
        ESP_LOGE(__func__, "Create task AllocateData failed.");
    }

#if (CONFIG_USING_WIFI)
    WIFI_initSTA();
#endif
}
