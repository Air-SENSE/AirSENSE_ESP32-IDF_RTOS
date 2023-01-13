<<<<<<< HEAD
set(app_src sht3x.c)
set(pre_req i2cdev log)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
=======
set(app_src sht3x.c)
set(pre_req i2cdev log)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
>>>>>>> 4093a5f80c80c982922bae844b0dc8c0c88f9ab3
                    REQUIRES ${pre_req})