#ifndef __PMS7003_H__
#define __PMS7003_H__

#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define START_CHARACTER_1   0x42
#define START_CHARACTER_2   0x4d
#define RX_BUFFER_SIZE      128

#define ID_PMS7003  0X02

#define ESP_ERROR_PMS7003_INIT_UART_FAILED          ((ID_PMS7003 << 12)|(0x00))
#define ESP_ERROR_PMS7003_SET_ACTIVE_MODE_FAILED    ((ID_PMS7003 << 12)|(0x01))
#define ESP_ERROR_PMS7003_READ_DATA_FAILED          ((ID_PMS7003 << 12)|(0x02))
#define PMS_ERROR_INVALID_VALUE                     UINT32_MAX

#define UART_CONFIG_DEFAULT()   {   .baud_rate = CONFIG_UART_BAUD_RATE,     \
                                    .data_bits = UART_DATA_8_BITS,          \
                                    .parity = UART_PARITY_DISABLE,          \
                                    .stop_bits = UART_STOP_BITS_1,          \
                                    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  \
                                    .source_clk = UART_SCLK_APB,            \
}

static const char pms7003_commandActiveMode[]  = {0x42,0x78,0x03,0x00,0x00,0x00,0x00,0x00,0xff};
static const char pms7003_commandPassiveMode[] = {0x42,0x78,0x04,0x00,0x00,0x00,0x00,0x00,0xff};

enum {
    indoor = 0,
    outdoor = 1,
};


/**
 * @brief 
 * 
 * @param uart_config 
 * @return esp_err_t 
 */
esp_err_t pms7003_initUart(uart_config_t *uart_config);

//static esp_err_t pms7003_checkCRC(uint8_t data[32]);

/**
 * @brief 
 * 
 * @return esp_err_t 
 */
esp_err_t pms7003_activeMode(void);

/**
 * @brief 
 * 
 * @param pms_modeAmbience 
 * @param pm1_0 
 * @param pm2_5 
 * @param pm10 
 * @return esp_err_t 
 */
esp_err_t pms7003_readData(const int pms_modeAmbience, uint32_t *pm1_0, uint32_t *pm2_5, uint32_t *pm10);

#endif