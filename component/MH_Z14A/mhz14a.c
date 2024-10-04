#include "mhz14a.h"

static uint8_t mhz14a_commandTemplate[9] = {
    0xff, // Start byte
    0x01, // Command byte
    0x00, // Data byte 1
    0x00, // Data byte 2
    0x00, // Data byte 3
    0x00, // Data byte 4
    0x00, // Data byte 5
    0x00, // Data byte 6
    0x79  // Checksum byte
};

/**
 * @brief Warms up the MHZ14A sensor.
 *
 * This function waits for the MHZ14A sensor to warm up before performing any operations.
 *
 * @return None
 */
static void mhz14a_warmUp()
{
    uint64_t time_start_warm_up = esp_timer_get_time();
    uint64_t time_end_warm_up = time_start_warm_up + TIME_TO_WARM_UP;
    // Wait until the warm up time has elapsed
    while (esp_timer_get_time() - time_start_warm_up < time_end_warm_up)
    {
        // Log a message indicating that the sensor is warming up
        ESP_LOGI(__func__, "MHZ14A warming up...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
}

/**
 * @brief Creates the capture queue for MH-Z14A sensor.
 *
 * This function initializes the capture queue used to store captured data from the MH-Z14A sensor.
 * The queue is created with a size of 1 element, where each element is of type uint32_t.
 * If the queue creation fails, an error message is logged and ESP_FAIL is returned.
 *
 * @return ESP_OK if the queue is successfully created, ESP_FAIL otherwise.
 */
static esp_err_t mhz14a_createCaptureQueue()
{
    // Create the capture queue with a size of 1 element, where each element is of type uint32_t
    mhz14a_captureQueue = xQueueCreate(1, sizeof(uint32_t));

    // Check if the queue creation was successful
    if (mhz14a_captureQueue == NULL)
    {
        // Log an error message
        ESP_LOGE(__func__, "Failed to allocate mhz14a_captureQueue!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**************************************************************************************************/

#ifdef CONFIG_MHZ14A_PWM

/**
 * @brief Callback function for MHZ14A sensor capture event
 *
 * @param cap_chan The capture channel handle
 * @param edata Pointer to the capture event data
 * @param user_data User data passed to the callback function
 * @return true if the task should be woken up, false otherwise
 */
static bool mhz14a_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t mhz14a_captureValueBeginOfSample = 0;
    static uint32_t mhz14a_captureValueEndOfSample = 0;
    BaseType_t high_task_wakeup = pdFALSE;

    // Calculate the interval in the ISR

    // Store the timestamp when a positive edge is detected
    if (edata->cap_edge == MCPWM_POS_EDGE)
    {
        mhz14a_captureValueBeginOfSample = edata->cap_value;
        mhz14a_captureValueEndOfSample = mhz14a_captureValueBeginOfSample;
    }
    else
    {
        // Store the timestamp when a negative edge is detected
        mhz14a_captureValueEndOfSample = edata->cap_value;

        // Calculate the pulse count
        uint32_t pulse_count = mhz14a_captureValueEndOfSample - mhz14a_captureValueBeginOfSample;

        // Send the measurement back through a queue
        xQueueSendFromISR(mhz14a_captureQueue, &pulse_count, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}
/**
 * @brief Initializes the PWM for MHZ14A sensor.
 *
 * @return esp_err_t Returns ESP_OK if successful, or an error code if initialization fails.
 */
esp_err_t mhz14a_initPWM()
{
    mhz14a_warmUp();
    if (mhz14a_createCaptureQueue() == ESP_FAIL)
    {
        return ESP_ERROR_MHZ14A_INIT_FAILED;
    }

    ESP_LOGI(__func__, "Install capture timer for MHZ14A sensor...");

    // Create a new capture timer for MHZ14A sensor
    mcpwm_cap_timer_handle_t mhz14a_captureTimer = NULL;
    mcpwm_capture_timer_config_t mhz14a_captureConfig = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_new_capture_timer(&mhz14a_captureConfig, &mhz14a_captureTimer));

    ESP_LOGI(__func__, "Install capture channel for MHZ14A sensor...");

    // Create a new capture channel for MHZ14A sensor
    mcpwm_cap_channel_handle_t mhz14a_captureChannel = NULL;
    mcpwm_capture_channel_config_t mhz14a_captureChannelConfig = {
        .gpio_num = CONFIG_MHZ14A_PWM_PIN,
        .prescale = 1,
        // capture on both edges
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_new_capture_channel(mhz14a_captureTimer, &mhz14a_captureChannelConfig, &mhz14a_captureChannel));

    ESP_LOGI(__func__, "Register capture callback");

    // Register capture callback
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = mhz14a_callback,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_capture_channel_register_event_callbacks(mhz14a_captureChannel, &cbs, NULL));

    ESP_LOGI(__func__, "Enable capture channel");

    // Enable capture channel
    ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_capture_channel_enable(mhz14a_captureChannel));

    ESP_LOGI(__func__, "Enable and start capture timer");

    // Enable and start capture timer
    ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_capture_timer_enable(mhz14a_captureTimer));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mcpwm_capture_timer_start(mhz14a_captureTimer));

    ESP_LOGI(__func__, "MHZ14A initialize success.");
    return ESP_OK;
}

esp_err_t mhz14a_readDataViaPWM(uint32_t *co2_ppm)
{
    // Write by PWM capture
    uint32_t pulse_count, time_high, time_low, pwm_ppm;

    // Block and wait for new measurement
    xQueueReceive(mhz14a_captureQueue, &pulse_count, portMAX_DELAY);

    // Calculate pulse width in microseconds
    uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());

    // Calculate time high and time low in milliseconds
    time_high = pulse_width_us / 1000;
    time_low = 1004 - time_high;

    // Calculate PWM concentration in parts per million
    pwm_ppm = co2_range0To5000 * (time_high - 2) / (time_high + time_low - 4);

    // Store the PWM concentration in the provided pointer
    *co2_ppm = pwm_ppm;

    return ESP_OK;
}

#endif
// #elif CONFIG_MHZ14A_UART

esp_err_t mhz14a_initUART(uart_config_t *uart_config)
{
    mhz14a_warmUp();

    ESP_LOGI(__func__, "MHZ14A initialize success.");

    /**
     * @note: 3rd parameter of uart_driver_install() function - rx_buffer_size never less than 128 byte.
     *        if rx_buffer_size <= 128, program print error code "uart rx buffer length error".
     */

    // Install UART driver with double the size of the RX buffer
    esp_err_t error_1 = uart_driver_install(CONFIG_MHZ14A_UART_PORT, (MHZ14A_UART_RX_BUFFER_SIZE * 2), 0, 0, NULL, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_1);

    // Configure UART parameters
    esp_err_t error_2 = uart_param_config(CONFIG_MHZ14A_UART_PORT, uart_config);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_2);

    // Set UART pins
    esp_err_t error_3 = uart_set_pin(CONFIG_MHZ14A_UART_PORT, CONFIG_MHZ14A_PIN_TX, CONFIG_MHZ14A_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_3);

    // Create a mutex for UART access
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
    }
    else
    {
        ESP_LOGE(__func__, "MHZ14A UART port initialize failed. (0x%x 0x%x 0x%x)", error_1, error_2, error_3);
        return ESP_ERROR_MHZ14A_INIT_FAILED;
    }
}

/**
 * Calculate the checksum for the received packet.
 *
 * @param packetReciveFromRXPin The packet received from the RX pin.
 * @return The calculated checksum.
 */
uint8_t mhz14a_getCheckSum(uint8_t *packetReciveFromRXPin)
{
    uint8_t i, checksum = 0;

    // Calculate the checksum by summing up the bytes in the packet.
    for (i = 1; i < 8; i++)
    {
        checksum += packetReciveFromRXPin[i];
    }

    // Invert the checksum and add 1.
    checksum = 0xff - checksum;
    checksum += 1;

    return checksum;
};

esp_err_t mhz14a_sendCommand(const uint32_t uart_command, bool on_off_function)
{
    // Create an array to store the command
    uint8_t arrayCommand[9] = {0};

    // Copy the command template into the array
    memcpy(arrayCommand, mhz14a_commandTemplate, sizeof(mhz14a_commandTemplate));

    // Set the UART command in the array
    arrayCommand[2] = uart_command;

    // Set the on/off function in the array based on the UART command
    arrayCommand[3] = (uart_command == MHZ14A_UART_CMD_SET_SELF_CALIBRATION_ZERO_POINT) ? (on_off_function ? 0xA0 : 0x00) : 0x00;

    // Calculate and set the checksum in the array
    arrayCommand[8] = mhz14a_getCheckSum(arrayCommand);

    // Send the command using the UART
    esp_err_t errorReturn = uart_write_bytes(CONFIG_MHZ14A_UART_PORT, (const uint8_t *)arrayCommand, 9);

    // Check if the command was sent successfully
    if (errorReturn == -1)
    {
        // Log an error message
        ESP_LOGE(__func__, "Sending command to MHZ14a sensor failed.");

        // Return an error code
        return ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED;
    }
    else
    {
        // Log a success message
        ESP_LOGI(__func__, "Sending command to MHZ14a sensor successful.");

        // Return a success code
        return ESP_OK;
    }
}

// Perform self calibration for the on/off function.
static esp_err_t mhz14a_on_off_functionSelfCalibration(bool statusFunction)
{
    // Send the command to set self-calibration zero point
    if (mhz14a_sendCommand(MHZ14A_UART_CMD_SET_SELF_CALIBRATION_ZERO_POINT, statusFunction) == ESP_OK)
    {
        // Log success and return ESP_OK
        ESP_LOGI(__func__, "%s function self-calibration for zero point successful.", statusFunction ? "ON" : "OFF");
        return ESP_OK;
    }
    else
    {
        // Log failure and return corresponding error code
        ESP_LOGE(__func__, "%s function self-calibration for zero point fail.", statusFunction ? "ON" : "OFF");
        return ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED;
    }
}

esp_err_t mhz14a_zeroPointCalibration(const bool statusFunction)
{
    // Send the command for calibration zero point
    esp_err_t errorReturn = mhz14a_sendCommand(MHZ14A_UART_CMD_CALIBRATION_ZERO_POINT, false);

    // Get the current tick count
    TickType_t startClibrationTime;
    startClibrationTime = xTaskGetTickCount();

    // Check if sending the command failed
    if (errorReturn == ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED)
    {
        ESP_LOGE(__func__, "Fail to send command for calibration zero point.");
        return ESP_ERROR_MHZ14A_CALIBRATION_ZERO_POINT;
    }
    else
    {
        // Delay for MHZ14A_TIME_FOR_CALIBRATION seconds
        xTaskDelayUntil(&startClibrationTime, MHZ14A_TIME_FOR_CALIBRATION / portTICK_PERIOD_MS);

        ESP_LOGI(__func__, "Calibration zero point successful.");
        return ESP_OK;
    }
};

esp_err_t mhz14a_spanPointCalibration()
{
    // Send command for calibration span point to MHZ14A
    esp_err_t errorReturn = mhz14a_sendCommand(MHZ14A_UART_CMD_CALIBRATION_SPAN_POINT, false);

    // Get the current tick count
    TickType_t startClibrationTime;
    startClibrationTime = xTaskGetTickCount();

    // Check if sending the command failed
    if (errorReturn == ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED)
    {
        // Log an error message
        ESP_LOGE(__func__, "Fail to send command for calibrating span point.");

        // Return an error code
        return ESP_ERROR_MHZ14A_CALIBRATION_ZERO_POINT;
    }
    else
    {
        // Delay the task until MHZ14A_TIME_FOR_CALIBRATION milliseconds have passed
        xTaskDelayUntil(&startClibrationTime, MHZ14A_TIME_FOR_CALIBRATION);

        // Log an info message
        ESP_LOGI(__func__, "Calibrate span point successfully.");

        // Return success code
        return ESP_OK;
    }
};

esp_err_t mhz14a_getDataFromSensorViaUART(uint32_t *co2_ppm)
{
    // Send command to MH-Z14A sensor to get concentration
    esp_err_t errorReturn = mhz14a_sendCommand(MHZ14A_UART_CMD_GET_CONCENTRATION, false);
    uart_wait_tx_done(CONFIG_MHZ14A_UART_PORT, 500 / portTICK_PERIOD_MS);

    // Check if command was sent successfully
    if (errorReturn != ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED)
    {
        uint8_t rawDataSensor[128];
        int lenghtRawDataSensor = 0;

        // Get the length of buffered data
        ESP_ERROR_CHECK(uart_get_buffered_data_len(CONFIG_MHZ14A_UART_PORT, (size_t *)&lenghtRawDataSensor));

        // Delay to ensure complete reception
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Read the received data
        lenghtRawDataSensor = uart_read_bytes(CONFIG_MHZ14A_UART_PORT, rawDataSensor, sizeof(rawDataSensor), 100 / portTICK_PERIOD_MS);

        // Delay to ensure complete reception
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Check if the length of the received data is valid
        if (lenghtRawDataSensor != 9)
        {
            ESP_LOGE(__func__, "Get raw data from TX buffer failed. %d", lenghtRawDataSensor);
            *co2_ppm = MHZ14A_ERROR_INVALID_VALUE;
            return ESP_ERROR_MHZ14A_GET_RAW_DATA_FORM_UART_FAILED;
        }

        // Check if the received data is correct
        if (rawDataSensor[0] == UART_DATA_RECIVE_START_CHARACTER_1 &&
            rawDataSensor[1] == UART_DATA_RECIVE_START_CHARACTER_2 &&
            rawDataSensor[8] == mhz14a_getCheckSum(rawDataSensor))
        {
            // Calculate CO2 concentration
            *co2_ppm = (uint32_t)(((rawDataSensor[2]) << 8) | rawDataSensor[3]); // Or *co2_ppm = rawDataSensor[2] * 256 + rawDataSensor[3];

            ESP_LOGI(__func__, "Raw data from TX buffer: [%u %u %u %u %u %u %u %u %u]", rawDataSensor[0],
                     rawDataSensor[1],
                     rawDataSensor[2],
                     rawDataSensor[3],
                     rawDataSensor[4],
                     rawDataSensor[5],
                     rawDataSensor[6],
                     rawDataSensor[7],
                     rawDataSensor[8]);
            // Return success
            return ESP_OK;
        }
        else
        {
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
    }
    else
    {
        *co2_ppm = MHZ14A_ERROR_INVALID_VALUE;
        ESP_LOGE(__func__, "Get CO2 concentration form MHZ14a Sensor failed.");
        return ESP_ERROR_MHZ14A_GET_CONCENTRATION_FAILED;
    }
};

esp_err_t mhz14a_setRangeSetting(const uint16_t co2_Range)
{
    // Initialize an array to store the command
    uint8_t arrayCommand[9] = {0};

    // Copy the command template to the array
    memcpy(arrayCommand, mhz14a_commandTemplate, sizeof(mhz14a_commandTemplate));

    // Set the detection range setting command
    arrayCommand[2] = MHZ14A_UART_CMD_DETECTION_RANGE_SETTING;

    // Set the CO2 range
    arrayCommand[7] |= co2_Range;
    arrayCommand[6] |= (co2_Range >> 8);

    // Calculate and set the checksum
    arrayCommand[8] = mhz14a_getCheckSum(arrayCommand);

    // Send the command to the MHZ14a sensor
    esp_err_t errorReturn = uart_write_bytes(CONFIG_MHZ14A_UART_PORT, (const uint8_t *)arrayCommand, 9);

    // Check if sending the command failed
    if (errorReturn == -1)
    {
        // Log the error and return the corresponding error code
        ESP_LOGE(__func__, "Sending command to MHZ14a sensor failed.");
        return ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED;
    }
    else
    {
        // Log the success and return ESP_OK
        ESP_LOGI(__func__, "Sending command to MHZ14a sensor successful.");
        return ESP_OK;
    }
};

// #endif  // CONFIG_MHZ14A_UART

/**************************************************************************************************/

#ifdef CONFIG_HD_PIN

esp_err_t mhz14a_autoCalibartionViaHDPin()
{
    // Reset the pin connected to the MHZ14a sensor
    gpio_reset_pin(CONFIG_HD_PIN);

    // Set the direction of the pin as output
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(CONFIG_HD_PIN, GPIO_MODE_OUTPUT));

    // Log the start of auto calibration
    ESP_LOGI(__func__, "MHZ14a sensor is starting auto calibration...");

    // Set the pin level to low
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CONFIG_HD_PIN, 0));

    // Log a message indicating the wait time for auto calibration
    ESP_LOGI(__func__, "Waiting about 7 seconds for auto calibration function...");

    // Delay the task execution for 7 seconds
    vTaskDelay((uint32_t)7000 / portTICK_PERIOD_MS);

    // Set the pin level to high
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CONFIG_HD_PIN, 1));

    // Log a message indicating the completion of calibration
    ESP_LOGI(__func__, "MHZ14a finished calibration!");

    // Reset the pin connected to the MHZ14a sensor
    gpio_reset_pin(CONFIG_HD_PIN);

    // Return ESP_OK to indicate successful execution
    return ESP_OK;
};

#endif // CONFIG_HD_PIN
