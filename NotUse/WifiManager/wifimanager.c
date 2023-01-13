#include "wifimanager.h"


// void WIFI_eventHandlerFunctionCallback(void* argument_t, esp_event_base_t event_base_t, int32_t event_id_t, void* event_data_t, WIFI_eventHandler_cb WIFI_eventHandler_t)
// {
//     return WIFI_eventHandler_t(argument_t, event_base_t, event_id_t, WIFI_eventHandler_t);
// }

esp_err_t WIFI_initSTA(const char *WIFI_ssid, const char *WIFI_password, EventGroupHandle_t *WIFI_eventGroupHandle_t, esp_event_handler_t WIFI_eventHandle_t)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t WIFI_initConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&WIFI_initConfig));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        WIFI_eventHandle_t,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        WIFI_eventHandle_t,
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

    EventBits_t bits = xEventGroupWaitBits(*WIFI_eventGroupHandle_t,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(__func__, "connected to ap SSID:%s password:%s",
                 CONFIG_SSID, CONFIG_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(__func__, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_SSID, CONFIG_PASSWORD);
    } else {
        ESP_LOGE(__func__, "UNEXPECTED EVENT");
    }
}

