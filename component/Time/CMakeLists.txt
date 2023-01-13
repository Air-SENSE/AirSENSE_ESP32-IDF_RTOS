set(app_src DS3231Time.c)
set(pre_req DS3231 log)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
