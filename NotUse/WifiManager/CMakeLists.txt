set(app_src wifimanager.c)
set(pre_req nvs_flash)
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
