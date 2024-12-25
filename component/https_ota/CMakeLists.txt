idf_component_register(SRCS "https_ota.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client bootloader_support esp_app_format esp_event esp_https_ota
                    PRIV_REQUIRES log app_update)
