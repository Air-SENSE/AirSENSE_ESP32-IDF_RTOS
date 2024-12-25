set(app_src ili9341_extended.c)
set(pre_req lvgl__lvgl espressif__esp_lcd_ili9341 driver log esp_timer)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
