set(app_src bme280.c)
set(pre_req i2cdev log esp_idf_lib_helpers bmp280)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})