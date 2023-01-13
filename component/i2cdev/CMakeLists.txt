set(app_src i2cdev.c)
set(pre_req driver freertos esp_idf_lib_helpers)
idf_component_register( SRCS ${app_src}
                        INCLUDE_DIRS "."
                        REQUIRES ${pre_req})