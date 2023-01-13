set(app_src bmp280.c)
set(pre_req i2cdev log esp_idf_lib_helpers)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})