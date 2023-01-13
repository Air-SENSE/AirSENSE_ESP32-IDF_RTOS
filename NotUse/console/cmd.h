#ifndef __CMD_H__
#define __CMD_H__

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

static const char* TAG = "CMD";
#define PROMPT CONFIG_IDF_TARGET



#endif