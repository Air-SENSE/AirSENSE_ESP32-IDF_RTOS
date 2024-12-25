/**
 * @file mhz14a.h
 * @author Nguyen Nhu Hai Long K65 HUST ( @long27032002 )
 * @brief MH-Z14A sensor driver
 * @version 1.1
 * @date 2023-03-26
 *
 * @copyright Copyright (c) 2023
 */

#ifndef __MHZ14A_H__
#define __MHZ14A_H__

#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "sdkconfig.h"

enum
{
    co2_range0To2000 = 2000,
    co2_range0To5000 = 5000,
    co2_range0To10000 = 10000,
};

#define ID_MHZ14A 0x0E
#define ESP_ERROR_MHZ14A_INIT_FAILED ((ID_MHZ14A << 12) | (0x00))
#define TIME_TO_WARM_UP 2000000 // Time in microseconds (3 minutes) đang để 18s để test dễ

#define MHZ14A_ERROR_INVALID_VALUE UINT32_MAX

__attribute__((unused)) static SemaphoreHandle_t mhz14a_uartMuxtex = NULL;

static QueueHandle_t mhz14a_captureQueue = NULL;

// Define for UART protocol
//  #if CONFIG_MHZ14A_UART
#define MHZ14A_UART_CONFIG_DEFAULT() {         \
    .baud_rate = CONFIG_MHZ14A_UART_BAUD_RATE, \
    .data_bits = UART_DATA_8_BITS,             \
    .stop_bits = UART_STOP_BITS_1,             \
    .parity = UART_PARITY_DISABLE,             \
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,     \
    .source_clk = UART_SCLK_APB,               \
};

#define MHZ14A_UART_RX_BUFFER_SIZE 128UL
#define ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED ((ID_MHZ14A << 12) | (0x01))
#define ESP_ERROR_MHZ14A_CALIBRATION_ZERO_POINT ((ID_MHZ14A << 12) | (0x02))
#define ESP_ERROR_MHZ14A_GET_CONCENTRATION_FAILED ((ID_MHZ14A << 12) | (0x03))
#define ESP_ERROR_MHZ14A_UART_RAW_DATA_NOT_CORRECT ((ID_MHZ14A << 12) | (0x04))
#define ESP_ERROR_MHZ14A_GET_RAW_DATA_FORM_UART_FAILED ((ID_MHZ14A << 12) | (0x05))

// UART data from datasheet, no need to change
#define MHZ14A_TIME_FOR_CALIBRATION ((uint32_t)7000 / portTICK_PERIOD_MS) // Time in milliseconds for calibration
#define MHZ14A_UART_CMD_GET_CONCENTRATION 0x86                            // Command to get gas concentration
#define MHZ14A_UART_CMD_CALIBRATION_ZERO_POINT 0x87                       // Command for zero point calibration
#define MHZ14A_UART_CMD_CALIBRATION_SPAN_POINT 0x88                       // Command for span point calibration
#define MHZ14A_UART_CMD_SET_SELF_CALIBRATION_ZERO_POINT 0x79              // Command to set self-calibration zero point
#define MHZ14A_UART_CMD_ON_SELT_CALIBRATION_ZERO_POINT 0xA0               // Command to turn on self-calibration zero point
#define MHZ14A_UART_CMD_OFF_SELT_CALIBRATION_ZERO_POINT 0x00              // Command to turn off self-calibration zero point
#define MHZ14A_UART_CMD_DETECTION_RANGE_SETTING 0x99                      // Command for detection range setting
#define UART_DATA_RECIVE_START_CHARACTER_1 0xff                           // Start character 1 for UART data receive
#define UART_DATA_RECIVE_START_CHARACTER_2 0x86                           // Start character 2 for UART data receive

/**
 * @brief Initialize the UART communication for MH-Z14A sensor.
 *
 * @param[out] uart_config Pointer to a `uart_config_t` structure containing the UART configuration.
 * @return `esp_err_t` value indicating success or failure.
 */
esp_err_t mhz14a_initUART(uart_config_t *uart_config);

/**
 * @brief Calculate and return the checksum of the received packet.
 *
 * @param[out] packetReciveFromRXPin Pointer to the received packet.
 * @return The calculated checksum.
 */
uint8_t mhz14a_getCheckSum(uint8_t *packetReciveFromRXPin);

/**
 * @brief Send a command to the MH-Z14A sensor.
 *
 * @param[in] uart_command The command to send.
 * @param[in] on_off_function The on/off function status.
 * @return `esp_err_t` value indicating success or failure.
 */
esp_err_t mhz14a_sendCommand(const uint32_t uart_command, bool on_off_function);

/**
 * @brief Perform self calibration for the on/off function.
 *
 * @param[in] statusFunction The status of the function.
 * @return `esp_err_t` value indicating success or failure.
 */
__attribute__((unused)) static esp_err_t mhz14a_on_off_functionSelfCalibration(bool statusFunction);

/**
 * @brief Perform zero point calibration for the MH-Z14A sensor.
 *
 * @param[in] statusFunction The status of the function.
 * @return `esp_err_t` value indicating success or failure.
 */
esp_err_t mhz14a_zeroPointCalibration(const bool statusFunction);

/**
 * @brief Perform span point calibration for the MH-Z14A sensor.
 *
 * @return `esp_err_t` value indicating success or failure.
 */
esp_err_t mhz14a_spanPointCalibration();

/**
 * @brief Get CO2 data from the MH-Z14A sensor via UART.
 *
 * @param[out] co2_ppm Pointer to a variable to store the CO2 value in parts per million.
 * @return `esp_err_t` value indicating success or failure.
 */
esp_err_t mhz14a_getDataFromSensorViaUART(uint32_t *co2_ppm);

/**
 * @brief Set the range setting for the MH-Z14A sensor.
 *
 * @param[in] co2_Range The CO2 range setting to set.
 * @return `esp_err_t` value indicating success or failure.
 */
esp_err_t mhz14a_setRangeSetting(const uint16_t co2_Range);

// #endif  // CONFIG_MHZ14A_UART

/**************************************************************************************************/

// Define for PWM protocol
#if CONFIG_MHZ14A_PWM
#include "driver/mcpwm_types_legacy.h"

#if ESP_IDF_VER <= ESP_IDF_VERSION_VAL(4, 4, 4)
#define MHZ14A_PWM_PIN_CONFIG_DEFAULT {         \
    .pin_bit_mask = BIT(CONFIG_MHZ14A_PWM_PIN), \
    .mode = GPIO_MODE_INPUT,                    \
    .pull_up_en = GPIO_PULLUP_ENABLE,           \
    .pull_down_en = GPIO_PULLDOWN_DISABLE,      \
    .intr_type = GPIO_INTR_ANYEDGE,             \
}
#endif // ESP_IDF_VER
#define ESP_ERROR_MHZ14A_READ_DATA_FAILED ((ID_MHZ14A << 12) | (0X06))
#define ESP_ERROR_MHZ14A_CONFIG_GPIO_FAILED ((ID_MHZ14A << 12) | (0x07))
#define ESP_ERROR_MHZ14A_INIT_INTR_FAILED ((ID_MHZ14A << 12) | (0x08))

/**
 * @brief Initialize the MH-Z14A PWM.
 *
 * This function initializes the MH-Z14A PWM.
 *
 * @return ESP_OK if the initialization is successful, otherwise an error code.
 */
esp_err_t mhz14a_initPWM();

/**
 * @brief Reads the CO2 concentration from MH-Z14A sensor using PWM capture.
 *
 * @param[out] co2_ppm Pointer to store the CO2 concentration in parts per million.
 * @return esp_err_t Returns ESP_OK on success, or an error code if an error occurred.
 */
esp_err_t mhz14a_readDataViaPWM(uint32_t *co2_ppm);

#endif // CONFIG_MHZ14A_PWM

/**************************************************************************************************/

#ifdef CONFIG_HD_PIN
esp_err_t mhz14a_autoCalibartionViaHDPin(void);
#endif // CONFIG_HD_PIN

/**************************************************************************************************/

#endif // __MHZ14A_H__
