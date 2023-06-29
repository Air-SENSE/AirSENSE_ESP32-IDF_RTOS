#include "sntp_sync.h"


void sntp_init_func()
{
    ESP_LOGI(__func__, "Initializing SNTP.");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();
}

esp_err_t sntp_setTime(struct tm *timeInfo, time_t *timeNow)
{
    for (size_t i = 0; (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) && (i < RETRY_SET_TIME_SYSTEM); i++)
    {
        ESP_LOGI(__func__, "Waiting for system time to be set...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    time(timeNow);
    localtime_r(timeNow, timeInfo);

    char timeString[64];

    // Set timezone to VietNam Standard Time
    setenv("TZ", "GMT-07", 1);
    tzset();
    localtime_r(timeNow, timeInfo);
    strftime(timeString, sizeof(timeString), "%c", timeInfo);
    ESP_LOGI(__func__, "The current date/time in Viet Nam is: %s ", timeString);
    return ESP_OK;
}
