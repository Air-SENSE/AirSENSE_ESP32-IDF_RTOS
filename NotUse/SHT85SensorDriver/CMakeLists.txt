set(app_src SHT85Driver.c)
set(pre_req sht3x log)
idf_component_register( SRCS ${app_src}
                        INCLUDE_DIRS "."
                        REQUIRES ${pre_req})
