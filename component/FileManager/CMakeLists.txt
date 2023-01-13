<<<<<<< HEAD
<<<<<<< HEAD
set(app_src sdcard.c)
set(pre_req vfs fatfs)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
=======
set(app_src sdcard.c)
set(pre_req )
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
>>>>>>> 4093a5f80c80c982922bae844b0dc8c0c88f9ab3
=======
set(app_src sdcard.c)
set(pre_req )
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
>>>>>>> 4093a5f80c80c982922bae844b0dc8c0c88f9ab3
