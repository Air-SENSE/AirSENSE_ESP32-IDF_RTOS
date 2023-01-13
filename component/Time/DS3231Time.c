#include "DS3231Time.h"

static const char *TAG = "DS3231";


esp_err_t ds3231_convertTimeToString(struct tm *time, char* timeString, const unsigned int lenghtString)
{
    memset(timeString, 0, lenghtString);
    int lenght = 0;
    lenght = sprintf(timeString, timeFormat, time->tm_hour, time->tm_min, time->tm_sec, time->tm_year, time->tm_mon, time->tm_mday);
    if (lenght)
    {
        ESP_LOGE(__func__, "Convert time to string fail.");
        return ESP_FAIL;
    }
    
    ESP_LOGI(__func__, "Convert time to string success.");
    return ESP_OK;
}

esp_err_t ds3231_getEpochTime(i2c_dev_t *dev, unsigned long *epochTime)
{
    struct tm currentTime;
    esp_err_t err_code = ESP_OK;
    err_code = ds3231_get_time(dev, &currentTime);
    if (err_code != ESP_OK)
    {
        ESP_LOGE(__func__, "DS3231 get UnixTime fail(%s).", esp_err_to_name(err_code));
    }

    *epochTime  = SECONDS_FROM_1970_TO_2022;
    *epochTime += (currentTime.tm_year - 2022) * SECONDS_PER_YEAR;
    *epochTime += currentTime.tm_yday * SECONDS_PER_DAY;
    *epochTime += currentTime.tm_hour * SECONDS_PER_HOUR;
    *epochTime += currentTime.tm_min  * SECONDS_PER_MIN;
    *epochTime += currentTime.tm_sec;
    ESP_LOGI(__func__, "DS3231 get EpochTime success.");
    
    return ESP_OK;
}
