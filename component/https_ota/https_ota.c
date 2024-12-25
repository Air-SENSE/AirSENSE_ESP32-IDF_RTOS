/*
 * SPDX-FileCopyrightText: 2017-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "https_ota.h"
#include "esp_https_ota.h"
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <errno.h>
#include <sys/param.h>
#include <inttypes.h>

#define IMAGE_HEADER_SIZE (1024)

#define DEFAULT_OTA_BUF_SIZE (IMAGE_HEADER_SIZE)

_Static_assert(DEFAULT_OTA_BUF_SIZE > (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t) + 1), "OTA data buffer too small");

#define DEFAULT_REQUEST_SIZE (64 * 1024)

static const int DEFAULT_MAX_AUTH_RETRIES = 10;

static const char *TAG = "esp_https_ota";
typedef struct esp_https_ota_handle esp_https_ota_t;
typedef enum
{
    ESP_HTTPS_OTA_INIT,
    ESP_HTTPS_OTA_BEGIN,
    ESP_HTTPS_OTA_IN_PROGRESS,
    ESP_HTTPS_OTA_SUCCESS,
} esp_https_ota_state;

static const char *ota_event_name_table[] = {
    "ESP_HTTPS_OTA_START",
    "ESP_HTTPS_OTA_CONNECTED",
    "ESP_HTTPS_OTA_GET_IMG_DESC",
    "ESP_HTTPS_OTA_VERIFY_CHIP_ID",
    "ESP_HTTPS_OTA_DECRYPT_CB",
    "ESP_HTTPS_OTA_WRITE_FLASH",
    "ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION",
    "ESP_HTTPS_OTA_FINISH",
    "ESP_HTTPS_OTA_ABORT",
};

struct esp_https_ota_handle
{
    esp_ota_handle_t update_handle;
    const esp_partition_t *update_partition;
    esp_http_client_handle_t http_client;
    char *ota_upgrade_buf;
    size_t ota_upgrade_buf_size;
    int binary_file_len;
    int image_length;
    int max_http_request_size;
    esp_https_ota_state state;
    bool bulk_flash_erase;
    bool partial_http_download;
    int max_authorization_retries;
#if CONFIG_ESP_HTTPS_OTA_DECRYPT_CB
    decrypt_cb_t decrypt_cb;
    void *decrypt_user_ctx;
#endif
};

static void _http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static bool redirection_required(int status_code)
{
    switch (status_code)
    {
    case HttpStatus_MovedPermanently:
    case HttpStatus_Found:
    case HttpStatus_SeeOther:
    case HttpStatus_TemporaryRedirect:
    case HttpStatus_PermanentRedirect:
        return true;
    default:
        return false;
    }
    return false;
}

static bool process_again(int status_code)
{
    switch (status_code)
    {
    case HttpStatus_MovedPermanently:
    case HttpStatus_Found:
    case HttpStatus_SeeOther:
    case HttpStatus_TemporaryRedirect:
    case HttpStatus_PermanentRedirect:
    case HttpStatus_Unauthorized:
        return true;
    default:
        return false;
    }
    return false;
}

static void esp_https_ota_dispatch_event(int32_t event_id, const void *event_data, size_t event_data_size)
{
    if (esp_event_post(ESP_HTTPS_OTA_EVENT, event_id, event_data, event_data_size, portMAX_DELAY) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to post https_ota event: %s", ota_event_name_table[event_id]);
    }
}
static esp_err_t _http_handle_response_code(esp_https_ota_t *https_ota_handle, int status_code)
{
    esp_err_t err;
    if (redirection_required(status_code))
    {
        err = esp_http_client_set_redirection(https_ota_handle->http_client);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "URL redirection Failed");
            return err;
        }
    }
    else if (status_code == HttpStatus_Unauthorized)
    {
        if (https_ota_handle->max_authorization_retries == 0)
        {
            ESP_LOGE(TAG, "Reached max_authorization_retries (%d)", status_code);
            return ESP_FAIL;
        }
        https_ota_handle->max_authorization_retries--;
        esp_http_client_add_auth(https_ota_handle->http_client);
    }
    else if (status_code == HttpStatus_NotFound || status_code == HttpStatus_Forbidden)
    {
        ESP_LOGE(TAG, "File not found(%d)", status_code);
        return ESP_FAIL;
    }
    else if (status_code >= HttpStatus_BadRequest && status_code < HttpStatus_InternalError)
    {
        ESP_LOGE(TAG, "Client error (%d)", status_code);
        return ESP_FAIL;
    }
    else if (status_code >= HttpStatus_InternalError)
    {
        ESP_LOGE(TAG, "Server error (%d)", status_code);
        return ESP_FAIL;
    }

    char upgrade_data_buf[256];
    // process_again() returns true only in case of redirection.
    if (process_again(status_code))
    {
        while (1)
        {
            /*
             *  In case of redirection, esp_http_client_read() is called
             *  to clear the response buffer of http_client.
             */
            int data_read = esp_http_client_read(https_ota_handle->http_client, upgrade_data_buf, sizeof(upgrade_data_buf));
            if (data_read <= 0)
            {
                return ESP_OK;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t _http_connect(esp_https_ota_t *https_ota_handle)
{
    esp_err_t err = ESP_FAIL;
    int status_code, header_ret;
    do
    {
        char *post_data = NULL;
        /* Send POST request if body is set.
         * Note: Sending POST request is not supported if partial_http_download
         * is enabled
         */
        int post_len = esp_http_client_get_post_field(https_ota_handle->http_client, &post_data);
        err = esp_http_client_open(https_ota_handle->http_client, post_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
            return err;
        }
        if (post_len)
        {
            int write_len = 0;
            while (post_len > 0)
            {
                write_len = esp_http_client_write(https_ota_handle->http_client, post_data, post_len);
                if (write_len < 0)
                {
                    ESP_LOGE(TAG, "Write failed");
                    return ESP_FAIL;
                }
                post_len -= write_len;
                post_data += write_len;
            }
        }
        header_ret = esp_http_client_fetch_headers(https_ota_handle->http_client);
        if (header_ret < 0)
        {
            return header_ret;
        }

        status_code = esp_http_client_get_status_code(https_ota_handle->http_client);
        err = _http_handle_response_code(https_ota_handle, status_code);
        if (err != ESP_OK)
        {
            return err;
        }
    } while (process_again(status_code));
    return err;
}

static bool is_server_verification_enabled(const esp_https_ota_config_t *ota_config)
{
    return (ota_config->http_config->cert_pem || ota_config->http_config->use_global_ca_store || ota_config->http_config->crt_bundle_attach != NULL);
}

esp_err_t esp_https_ota_begin_with_api_key(const esp_https_ota_config_t *ota_config, esp_https_ota_handle_t *handle, char *api_key)
{
    esp_https_ota_dispatch_event(ESP_HTTPS_OTA_START, NULL, 0);

    esp_err_t err;

    if (handle == NULL || ota_config == NULL || ota_config->http_config == NULL)
    {
        ESP_LOGE(TAG, "esp_https_ota_begin: Invalid argument");
        if (handle)
        {
            *handle = NULL;
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_server_verification_enabled(ota_config))
    {
#if CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP
        ESP_LOGW(TAG, "Continuing with insecure option because CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP is set.");
#else
        ESP_LOGE(TAG, "No option for server verification is enabled in esp_http_client config.");
        *handle = NULL;
        return ESP_ERR_INVALID_ARG;
#endif
    }

    esp_https_ota_t *https_ota_handle = calloc(1, sizeof(esp_https_ota_t));
    if (!https_ota_handle)
    {
        ESP_LOGE(TAG, "Couldn't allocate memory to upgrade data buffer");
        *handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    https_ota_handle->partial_http_download = ota_config->partial_http_download;
    https_ota_handle->max_http_request_size = (ota_config->max_http_request_size == 0) ? DEFAULT_REQUEST_SIZE : ota_config->max_http_request_size;
    https_ota_handle->max_authorization_retries = ota_config->http_config->max_authorization_retries;

    if (https_ota_handle->max_authorization_retries == 0)
    {
        https_ota_handle->max_authorization_retries = DEFAULT_MAX_AUTH_RETRIES;
    }
    else if (https_ota_handle->max_authorization_retries == -1)
    {
        https_ota_handle->max_authorization_retries = 0;
    }

    /* Initiate HTTP Connection */
    https_ota_handle->http_client = esp_http_client_init(ota_config->http_config);
    if (https_ota_handle->http_client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        err = ESP_FAIL;
        goto failure;
    }
    esp_http_client_set_post_field(https_ota_handle->http_client, api_key, strlen(api_key));
    esp_http_client_set_method(https_ota_handle->http_client, HTTP_METHOD_POST);
    if (ota_config->http_client_init_cb)
    {
        err = ota_config->http_client_init_cb(https_ota_handle->http_client);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "http_client_init_cb returned 0x%x", err);
            goto http_cleanup;
        }
    }

    if (https_ota_handle->partial_http_download)
    {
        esp_http_client_set_method(https_ota_handle->http_client, HTTP_METHOD_HEAD);
        err = esp_http_client_perform(https_ota_handle->http_client);
        if (err == ESP_OK)
        {
            int status = esp_http_client_get_status_code(https_ota_handle->http_client);
            if (status != HttpStatus_Ok)
            {
                ESP_LOGE(TAG, "Received incorrect http status %d", status);
                err = ESP_FAIL;
                goto http_cleanup;
            }
        }
        else
        {
            ESP_LOGE(TAG, "ESP HTTP client perform failed: %d", err);
            goto http_cleanup;
        }

        https_ota_handle->image_length = esp_http_client_get_content_length(https_ota_handle->http_client);
        esp_http_client_close(https_ota_handle->http_client);

        if (https_ota_handle->image_length > https_ota_handle->max_http_request_size)
        {
            char *header_val = NULL;
            asprintf(&header_val, "bytes=0-%d", https_ota_handle->max_http_request_size - 1);
            if (header_val == NULL)
            {
                ESP_LOGE(TAG, "Failed to allocate memory for HTTP header");
                err = ESP_ERR_NO_MEM;
                goto http_cleanup;
            }
            esp_http_client_set_header(https_ota_handle->http_client, "Range", header_val);
            free(header_val);
        }
        esp_http_client_set_method(https_ota_handle->http_client, HTTP_METHOD_GET);
    }

    err = _http_connect(https_ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to establish HTTP connection");
        goto http_cleanup;
    }
    else
    {
        esp_https_ota_dispatch_event(ESP_HTTPS_OTA_CONNECTED, NULL, 0);
    }

    if (!https_ota_handle->partial_http_download)
    {
        https_ota_handle->image_length = esp_http_client_get_content_length(https_ota_handle->http_client);
    }

    https_ota_handle->update_partition = NULL;
    ESP_LOGI(TAG, "Starting OTA...");
    https_ota_handle->update_partition = esp_ota_get_next_update_partition(NULL);
    if (https_ota_handle->update_partition == NULL)
    {
        ESP_LOGE(TAG, "Passive OTA partition not found");
        err = ESP_FAIL;
        goto http_cleanup;
    }
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32,
             https_ota_handle->update_partition->subtype, https_ota_handle->update_partition->address);

    const int alloc_size = MAX(ota_config->http_config->buffer_size, DEFAULT_OTA_BUF_SIZE);
    https_ota_handle->ota_upgrade_buf = (char *)malloc(alloc_size);
    if (!https_ota_handle->ota_upgrade_buf)
    {
        ESP_LOGE(TAG, "Couldn't allocate memory to upgrade data buffer");
        err = ESP_ERR_NO_MEM;
        goto http_cleanup;
    }
#if CONFIG_ESP_HTTPS_OTA_DECRYPT_CB
    if (ota_config->decrypt_cb == NULL)
    {
        err = ESP_ERR_INVALID_ARG;
        goto http_cleanup;
    }
    https_ota_handle->decrypt_cb = ota_config->decrypt_cb;
    https_ota_handle->decrypt_user_ctx = ota_config->decrypt_user_ctx;
#endif
    https_ota_handle->ota_upgrade_buf_size = alloc_size;
    https_ota_handle->bulk_flash_erase = ota_config->bulk_flash_erase;
    https_ota_handle->binary_file_len = 0;
    *handle = (esp_https_ota_handle_t)https_ota_handle;
    https_ota_handle->state = ESP_HTTPS_OTA_BEGIN;
    return ESP_OK;

http_cleanup:
    _http_cleanup(https_ota_handle->http_client);
failure:
    free(https_ota_handle);
    *handle = NULL;
    return err;
}
