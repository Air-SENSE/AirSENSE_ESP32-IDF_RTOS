set(app_src button.c)
set(pre_req driver esp_timer)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
