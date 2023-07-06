#include "mhz14a.h"


uint8_t mhz14a_commandTemplate[9] = {0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };

static uint32_t cap_val_begin_of_sample = 0;
static uint32_t cap_val_end_of_sample = 0;

static xQueueHandle cap_queue;

static bool mhz_isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata,
                                  void *arg) {
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    BaseType_t high_task_wakeup = pdFALSE;
    if (edata->cap_edge == MCPWM_POS_EDGE) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t pulse_count = cap_val_end_of_sample - cap_val_begin_of_sample;
        // send measurement back though queue
        xQueueSendFromISR(cap_queue, &pulse_count, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

esp_err_t mhz14a_initPWM()
{
    cap_queue = xQueueCreate(1, sizeof(uint32_t));
    if (cap_queue == NULL) {
        ESP_LOGE(__func__, "failed to alloc cap_queue");
        return ESP_FAIL;
    }

    // set CAP_0 on GPIO
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, CONFIG_MHZ14A_PWM_PIN));
    // enable pull down CAP0, to reduce noise
    ESP_ERROR_CHECK(gpio_pulldown_en(CONFIG_MHZ14A_PWM_PIN));
    // enable both edge capture on CAP0
    mcpwm_capture_config_t conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = mhz_isr_handler,
        .user_data = NULL
    };
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &conf));
    ESP_LOGI(__func__, "MHZ14A PWM pin configured");

    //Warming up
    uint64_t time_start_warm_up = esp_timer_get_time();
    while (esp_timer_get_time() - time_start_warm_up <TIME_TO_WARM_UP)
    {
        ESP_LOGI(__func__,"MHZ14A warming up...");
        vTaskDelay(1000/portTICK_RATE_MS);
    }
    

    ESP_LOGI(__func__,"MHZ14A initialize success.");
    return ESP_OK;
}

esp_err_t mhz14a_readDataViaPWM(uint32_t *co2_ppm)
{

    //Wirte by PWM capture
    uint32_t pulse_count, time_high, time_low, pwm_ppm;
        // block and wait for new measurement
        xQueueReceive(cap_queue, &pulse_count, portMAX_DELAY);
        uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());
        //ESP_LOGI(__func__,"Time pulse high = %u",pulse_count);
        time_high = pulse_width_us/1000;
        time_low = 1004 - time_high;
        pwm_ppm = co2_range0To5000 * (time_high-2)/(time_high+time_low - 4);
        *co2_ppm = pwm_ppm;
        //ESP_LOGI(__func__, "Pulse width: %d, Co2 concentration: %dppm", pulse_width_us, pwm_ppm);
        return ESP_OK;
}

esp_err_t mhz14a_initUART(uart_config_t *uart_config)
{
    /** 
     * @note: 3rd parameter of uart_driver_install() function - rx_buffer_size never less than 128 byte.
     *        if rx_buffer_size <= 128, program print error code "uart rx buffer length error".
     */
    esp_err_t error_1 = uart_driver_install(CONFIG_MHZ14A_UART_PORT, (MHZ14A_UART_RX_BUFFER_SIZE * 2), 0, 0, NULL, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_1);
    esp_err_t error_2 = uart_param_config(CONFIG_MHZ14A_UART_PORT, uart_config);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_2);    
    esp_err_t error_3 = uart_set_pin(CONFIG_MHZ14A_UART_PORT, CONFIG_MHZ14A_PIN_TX, CONFIG_MHZ14A_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_3);

    mhz14a_uartMuxtex = xSemaphoreCreateMutex();

    if (mhz14a_uartMuxtex == NULL)
    {
        ESP_LOGE(__func__, "mhz14a_uartMuxtex semaphore create failed. (%d)", __LINE__);
        return ESP_FAIL;
    }

    if (error_1 == ESP_OK && 
        error_2 == ESP_OK &&
        error_3 == ESP_OK)
    {
        ESP_LOGI(__func__, "MHZ14A UART port initialize successful.");
        return ESP_OK;
    } else {
        ESP_LOGE(__func__, "MHZ14A UART port initialize failed. (0x%x 0x%x 0x%x)", error_1, error_2, error_3);
        return ESP_ERROR_MHZ14A_INIT_UART_FAILED;
    }
};

 uint8_t mhz14a_getCheckSum(uint8_t *packetReciveFromRXPin)
{
    uint8_t i, checksum = 0;
    for( i = 1; i < 8; i++)
    {
        checksum += packetReciveFromRXPin[i];
    }
    checksum = 0xff - checksum; 
    checksum += 1;
    return checksum;
};
/*
* @brief: Pham Van Sang test OKe
*/
 esp_err_t mhz14a_sendCommand(const uint32_t uart_command, bool on_off_function)
{
    uint8_t arrayCommand[9] = {0};
    memcpy(arrayCommand, mhz14a_commandTemplate, sizeof(mhz14a_commandTemplate));
    arrayCommand[2] = uart_command;
    arrayCommand[3] = (uart_command == MHZ14A_UART_CMD_SET_SELF_CALIBRATION_ZERO_POINT) ? (on_off_function ? 0xA0 : 0x00) : 0x00;
    arrayCommand[8] = mhz14a_getCheckSum(arrayCommand);
    esp_err_t errorReturn = uart_write_bytes(CONFIG_MHZ14A_UART_PORT, (const uint8_t*)arrayCommand, 9);
    if (errorReturn == -1)
    {
        ESP_LOGE(__func__, "Sending command to MHZ14a sensor failed.");
        return ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED;
    } else {
        ESP_LOGI(__func__, "Sending command to MHZ14a sensor successful.");
        return ESP_OK;
    }
};

__attribute__((unused)) static esp_err_t mhz14a_on_off_functionSelfCalibration(bool statusFunction)
{
    if (mhz14a_sendCommand(MHZ14A_UART_CMD_SET_SELF_CALIBRATION_ZERO_POINT, statusFunction) == ESP_OK)
    {
        ESP_LOGI(__func__, "%s function self-calibration for zero point successful.", statusFunction ? "ON" : "OFF");
        return ESP_OK;
    } else {
        ESP_LOGE(__func__, "%s function self-calibration for zero point fail.", statusFunction ? "ON" : "OFF");
        return ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED;
    }
};

esp_err_t mhz14a_zeroPointCalibration(const bool statusFunction)
{
    esp_err_t errorReturn = mhz14a_sendCommand(MHZ14A_UART_CMD_CALIBRATION_ZERO_POINT, false);

    TickType_t startClibrationTime;
    startClibrationTime = xTaskGetTickCount();

    if (errorReturn == ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED)
    {
        ESP_LOGE(__func__, "Fail to send command for calibration zero point.");
        return ESP_ERROR_MHZ14A_CALIBRATION_ZERO_POINT;
    } else {
        xTaskDelayUntil(&startClibrationTime, 7000 / portTICK_RATE_MS);
        ESP_LOGI(__func__, "Calibration zero point successful.");
        return ESP_OK;
    }
};

esp_err_t mhz14a_spanPointCalibration()
{
    esp_err_t errorReturn = mhz14a_sendCommand(MHZ14A_UART_CMD_CALIBRATION_SPAN_POINT, false);

    TickType_t startClibrationTime;
    startClibrationTime = xTaskGetTickCount();

    if (errorReturn == ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED)
    {
        ESP_LOGE(__func__, "Fail to send command for calibration span point.");
        return ESP_ERROR_MHZ14A_CALIBRATION_ZERO_POINT;
    } else {
        xTaskDelayUntil(&startClibrationTime, 7000 / portTICK_RATE_MS);
        ESP_LOGI(__func__, "Calibration span point successful.");
        return ESP_OK;
    }
};
// test Oke
esp_err_t mhz14a_getDataFromSensorViaUART(uint32_t *co2_ppm)
{
    esp_err_t errorReturn = mhz14a_sendCommand(MHZ14A_UART_CMD_GET_CONCENTRATION, false);
    uart_wait_tx_done(CONFIG_MHZ14A_UART_PORT, 500 / portTICK_RATE_MS);
    if (errorReturn != ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED)
    {
        uint8_t rawDataSensor[128];
        int lenghtRawDataSensor = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_MHZ14A_UART_PORT, (size_t*)&lenghtRawDataSensor));
        vTaskDelay(100/portTICK_RATE_MS);
        lenghtRawDataSensor = uart_read_bytes(CONFIG_MHZ14A_UART_PORT, rawDataSensor, sizeof(rawDataSensor), 100 / portTICK_RATE_MS);
        
        vTaskDelay(100/portTICK_RATE_MS);
        if (lenghtRawDataSensor != 9)
        {
            ESP_LOGE(__func__, "Get raw data from TX buffer failed.");
            *co2_ppm = MHZ14A_ERROR_INVALID_VALUE;
            return ESP_ERROR_MHZ14A_GET_RAW_DATA_FORM_UART_FAILED;
        }

        if (rawDataSensor[0] == UART_DATA_RECIVE_START_CHARACTER_1 &&
            rawDataSensor[1] == UART_DATA_RECIVE_START_CHARACTER_2 &&
            rawDataSensor[8] == mhz14a_getCheckSum(rawDataSensor))
        {
            //*co2_ppm = ((*co2_ppm | rawDataSensor[2]) << 8) | rawDataSensor[3];     // Or *co2_ppm = rawDataSensor[2] * 256 + rawDataSensor[3];
            *co2_ppm = rawDataSensor[2] * 256 + rawDataSensor[3];
            //ESP_LOGI(__func__, "Get CO2 concentration form MHZ14a Sensor successful.  CO2: %u[ppm]", *co2_ppm);
            return ESP_OK;
        } else {
            ESP_LOGE(__func__, "Get raw data from TX buffer success but not correct.");
            ESP_LOGE(__func__, "Raw data from TX buffer: [%u %u %u %u %u %u %u %u %u]", rawDataSensor[0],
                                                                                        rawDataSensor[1],
                                                                                        rawDataSensor[2],
                                                                                        rawDataSensor[3],
                                                                                        rawDataSensor[4],
                                                                                        rawDataSensor[5],
                                                                                        rawDataSensor[6],
                                                                                        rawDataSensor[7],
                                                                                        rawDataSensor[8]);
            *co2_ppm = MHZ14A_ERROR_INVALID_VALUE;
            return ESP_ERROR_MHZ14A_UART_RAW_DATA_NOT_CORRECT;
        }
    } else {
        *co2_ppm = MHZ14A_ERROR_INVALID_VALUE;
        ESP_LOGE(__func__, "Get CO2 concentration form MHZ14a Sensor failed.");
        return ESP_ERROR_MHZ14A_GET_CONCENTRATION_FAILED;
    }
};

esp_err_t mhz14a_setRangeSetting(const uint16_t co2_Range)
{
    uint8_t arrayCommand[9] = {0};
    memcpy(arrayCommand, mhz14a_commandTemplate, sizeof(mhz14a_commandTemplate));
    arrayCommand[2] = MHZ14A_UART_CMD_DETECTION_RANGE_SETTING;
    arrayCommand[7] |= co2_Range;
    arrayCommand[6] |= (co2_Range >> 8);
    arrayCommand[8] = mhz14a_getCheckSum(arrayCommand);
    esp_err_t errorReturn = uart_write_bytes(CONFIG_MHZ14A_UART_PORT, (const uint8_t*)arrayCommand, 9);
    if (errorReturn == -1)
    {
        ESP_LOGE(__func__, "Sending command to MHZ14a sensor failed.");
        return ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED;
    } else {
        ESP_LOGI(__func__, "Sending command to MHZ14a sensor successful.");
        return ESP_OK;
    }
};

//#endif

#ifdef CONFIG_HD_PIN
esp_err_t mhz14a_autoCalibartionViaHDPin()
{
    gpio_reset_pin(CONFIG_HD_PIN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(CONFIG_HD_PIN, GPIO_MODE_OUTPUT));
    ESP_LOGI(__func__, "MHZ14a sensor is starting auto calibration...");
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CONFIG_HD_PIN, 0));
    ESP_LOGI(__func__, "Waitting about 7 seconds for auto calibration function...");
    vTaskDelay((uint32_t)7000 / portTICK_RATE_MS);
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CONFIG_HD_PIN, 1));
    ESP_LOGI(__func__, "MHZ14a finished calibration!");
    gpio_reset_pin(CONFIG_HD_PIN);
    return ESP_OK;
};
#endif  
