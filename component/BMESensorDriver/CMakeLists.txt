<<<<<<< HEAD
set(app_src BME280SensorDriver.c)
set(pre_req i2cdev log esp_idf_lib_helpers BMP280)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
=======
set(app_src BME280SensorDriver.c)
set(pre_req i2cdev log esp_idf_lib_helpers BMP280)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
>>>>>>> 4093a5f80c80c982922bae844b0dc8c0c88f9ab3
                    REQUIRES ${pre_req})