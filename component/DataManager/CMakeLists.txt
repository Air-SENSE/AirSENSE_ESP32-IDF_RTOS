set(app_src datamanager.c)
set(pre_req log )
idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
                    