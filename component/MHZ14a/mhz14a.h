/**
 * @file mhz14a.h
 * @author Nguyen Nhu Hai Long K65 HUST ( @long27032002 )
 * @brief 
 * @version 0.1
 * @date 2023-03-26
 * 
 * @copyright Copyright (c) 2023
 * 
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

__attribute__((unused)) static const char *TAG = "MHZ14a";

enum {
    co2_range0To2000 = 2000,
    co2_range0To5000 = 5000,
    co2_range0To10000 = 10000,
};

///define pwm
#define GPIO_HIGH_LEVEL 1
#define GPIO_LOW_LEVEL 0

#define TIME_TO_WARM_UP 100000 // Time in microseconds

#define MHZ14A_PWM_PIN_CONFIG_DEFAULT { .pin_bit_mask = BIT(CONFIG_MHZ14A_PWM_PIN),  \
                                        .mode = GPIO_MODE_INPUT,                     \
                                        .pull_up_en = GPIO_PULLUP_ENABLE,            \
                                        .pull_down_en = GPIO_PULLDOWN_DISABLE,       \
                                        .intr_type = GPIO_INTR_ANYEDGE,              \
}
// esp_err_t mhz14a_initPWM(const gpio_config_t *mhz14a_pwm_pin_config);
// esp_err_t mhz14a_getDataFromSensorViaPWM(uint32_t *co2_ppm);

//#if (CONFIG_MHZ14A_PWM)
// xQueueHandle pwwmCapture_queue = NULL;

// esp_err_t mhz14a_initPWM();

// static void IRAM_ATTR mhz14a_pwmCapture_irs(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg);

// esp_err_t mhz14a_getDataFromSensor(uint32_t *co2_ppm);
////#elif (CONFIG_MHZ14A_UART)

#define MHZ14A_TIME_FOR_CALIBRATION                         ((uint32_t)7000 / portTICK_RATE_MS)
#define MHZ14A_UART_CMD_GET_CONCENTRATION                   0x86
#define MHZ14A_UART_CMD_CALIBRATION_ZERO_POINT              0x87
#define MHZ14A_UART_CMD_CALIBRATION_SPAN_POINT              0x88
#define MHZ14A_UART_CMD_SET_SELF_CALIBRATION_ZERO_POINT     0x79
#define MHZ14A_UART_CMD_ON_SELT_CALIBRATION_ZERO_POINT      0xA0
#define MHZ14A_UART_CMD_OFF_SELT_CALIBRATION_ZERO_POINT     0x00
#define MHZ14A_UART_CMD_DETECTION_RANGE_SETTING             0x99
#define UART_DATA_RECIVE_START_CHARACTER_1                  0xff
#define UART_DATA_RECIVE_START_CHARACTER_2                  0x86




#define ID_MHZ14A  0x0E
#define MHZ14A_UART_RX_BUFFER_SIZE 128UL
#define ESP_ERROR_MHZ14A_INIT_UART_FAILED                       ((ID_MHZ14A << 12)|(0x00))
#define ESP_ERROR_MHZ14A_UART_SEND_CMD_FAILED                   ((ID_MHZ14A << 12)|(0x01))
#define ESP_ERROR_MHZ14A_CALIBRATION_ZERO_POINT                 ((ID_MHZ14A << 12)|(0x02))
#define ESP_ERROR_MHZ14A_GET_CONCENTRATION_FAILED               ((ID_MHZ14A << 12)|(0x03))
#define ESP_ERROR_MHZ14A_UART_RAW_DATA_NOT_CORRECT              ((ID_MHZ14A << 12)|(0x04))
#define ESP_ERROR_MHZ14A_GET_RAW_DATA_FORM_UART_FAILED          ((ID_MHZ14A << 12)|(0x05))
//Define for PWM protocol
#define ESP_ERROR_MHZ14A_READ_DATA_FAILED                       ((ID_MHZ14A << 12)|(0X06))
#define ESP_ERROR_MHZ14A_CONFIG_GPIO_FAILED                     ((ID_MHZ14A << 12)|(0x07))
#define ESP_ERROR_MHZ14A_INIT_INTR_FAILED                       ((ID_MHZ14A << 12)|(0x08))
#define MHZ14A_ERROR_INVALID_VALUE                              UINT32_MAX


#define MHZ14A_UART_CONFIG_DEFAULT()   {.baud_rate = CONFIG_MHZ14A_UART_BAUD_RATE,     \
                                        .data_bits = UART_DATA_8_BITS,          \
                                        .stop_bits = UART_STOP_BITS_1,          \
                                        .parity = UART_PARITY_DISABLE,          \
                                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  \
                                        .source_clk = UART_SCLK_APB,            \
};

esp_err_t mhz14a_initPWM();
esp_err_t mhz14a_readDataViaPWM(uint32_t *co2_ppm);
static SemaphoreHandle_t mhz14a_uartMuxtex = NULL;

extern esp_err_t mhz14a_initUART(uart_config_t *uart_config);

esp_err_t mhz14a_sendCommand(const uint32_t uart_command, bool on_off_function);

esp_err_t mhz14a_zeroPointCalibration(const bool statusFunction);

esp_err_t mhz14a_spanPointCalibration(void);

esp_err_t mhz14a_detectionRangeSetting(void);

esp_err_t mhz14a_getDataFromSensorViaUART(uint32_t*);
//#endif


#ifdef CONFIG_HD_PIN
esp_err_t mhz14a_autoCalibartionViaHDPin(void);

#endif

#endif
