set(app_src OTA.c)
set(pre_req log app_update esp_http_client esp_https_ota efuse esp_common)
idf_component_register( SRCS ${app_src}
                        INCLUDE_DIRS "."
                        REQUIRES ${pre_req}
                        EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
