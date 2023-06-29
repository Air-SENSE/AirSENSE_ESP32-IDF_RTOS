set(app_src sntp_sync.c)
set(pre_req lwip log)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})