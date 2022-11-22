set(app_src BME280SensorDriver.c)
set(pre_req i2cdev log esp_idf_lib_helpers BMP280)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})