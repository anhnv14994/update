#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_manager.h"
#include "web_server.h"
#include "i2s_output.h"
#include "esp_raop_receiver.h"
#include "config_manager.h"
#include "dsp_engine.h"
#include "stereo_spatial.h"
#include "dsp_api.h"
#include "web_log.h"
#include "esp_mac.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "wled_sync.h"


static const char *TAG = "main";

static httpd_handle_t web_server = NULL;
static raop_handle_t *raop_handle = NULL;
static struct udp_pcb *dns_pcb = NULL;

// PCM tap runs PEQ + Analog Character + subsonic HPF + post-Character float
// volume/master in the DSP task. Stereo Spatial then runs before I2S output.
// RAOP now buffers decoded full-scale PCM so AirPlay volume does not quantize
// the signal before Character processing.
stereo_spatial_ctx_t *g_stereo_ctx = NULL;


static void dsp_pcm_callback(
    const uint8_t *data,
    size_t len,
    void *user_ctx)
{
    (void)user_ctx;

    /*
     * audio_buffer passes the writable DSP-ring frame here.
     * Process it directly: no scratch-to-frame memcpy on the hot path.
     * The callback type is const for API compatibility with older users.
     */
    if (data && len >= 4) {
        dsp_engine_process_inplace((uint8_t *)data, len);
    }
}

// static void audio_output_callback(const uint8_t *data, size_t len, void *user_data)
// {
//     // 1. Process PEQ / DSP Engine
//     size_t peq_len = len;
//     const uint8_t *peq_out = dsp_engine_process(data, len, &peq_len);

//     // 2. Process Stereo Spatial v2 (In-place processing)
//     if (g_stereo_ctx && peq_out) {
//         // Stereo 16-bit = 4 bytes/frame -> frames = peq_len / 4
//         size_t frames = peq_len / (sizeof(int16_t) * 2);
//         stereo_spatial_process_i16(g_stereo_ctx, (int16_t *)(uintptr_t)peq_out, frames);
//     }

//     // 3. Xuất ra I2S
//     i2s_output_write(peq_out, peq_len);
// }

static void audio_output_callback(const uint8_t *data, size_t len, void *user_data)
{
    // Dữ liệu 'data' đã được dsp_pcm_callback xử lý hoàn tất tại audio_dsp_task
    size_t out_len = len;

    // 1. Process Stereo Spatial (nếu bật)
    if (g_stereo_ctx && data) {
        size_t frames = out_len / (sizeof(int16_t) * 2);
        stereo_spatial_process_i16(g_stereo_ctx, (int16_t *)(uintptr_t)data, frames);
    }

    // 2. Xuất thẳng ra I2S
    i2s_output_write(data, out_len);
}

static void raop_volume_callback(float volume, void *user_ctx)
{
    (void)user_ctx;
    /* DSP uses this value only for optional loudness tracking. */
    (void)dsp_engine_set_output_volume(volume);
    /* Actual AirPlay gain is applied after Stereo Spatial, at final I2S output. */
    i2s_output_set_volume(volume);
}

static void raop_event_handler(raop_event_t event, void *event_data, void *user_ctx)
{
    switch (event) {
        case RAOP_EVENT_CONNECTED:
            ESP_LOGI(TAG, "AirPlay: client connected");
            break;
        case RAOP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "AirPlay: client disconnected");
            break;
        case RAOP_EVENT_BUFFERING:
            ESP_LOGI(TAG, "AirPlay: buffering");
            break;
        case RAOP_EVENT_PLAYING:
            ESP_LOGI(TAG, "AirPlay: playing");
            break;
        case RAOP_EVENT_PAUSED:
            ESP_LOGI(TAG, "AirPlay: paused");
            break;
        case RAOP_EVENT_STOPPED:
            ESP_LOGI(TAG, "AirPlay: stopped");
            break;
        case RAOP_EVENT_VOLUME: {
            float volume = *(float *)event_data;
            ESP_LOGI(TAG, "AirPlay: volume %.2f", volume);
            break;
        }
        case RAOP_EVENT_METADATA: {
            raop_metadata_t *meta = (raop_metadata_t *)event_data;
            ESP_LOGI(TAG, "AirPlay: %s - %s", meta->artist, meta->title);
            break;
        }
        case RAOP_EVENT_ARTWORK: {
            raop_artwork_t *art = (raop_artwork_t *)event_data;
            ESP_LOGI(TAG, "AirPlay: artwork %zu bytes", art->len);
            break;
        }
        case RAOP_EVENT_PROGRESS: {
            raop_progress_t *progress = (raop_progress_t *)event_data;
            ESP_LOGI(TAG, "AirPlay: progress %lu/%lu ms",
                     progress->current_ms, progress->total_ms);
            break;
        }
        case RAOP_EVENT_STALLED:
            ESP_LOGW(TAG, "AirPlay: stream stalled");
            break;
        default:
            break;
    }
}

static void start_airplay_receiver(const app_config_t *app_config)
{
    ESP_LOGI(TAG, "WLED enabled: %d, host: %s, port: %u",
             app_config->wled_enabled, app_config->wled_host, app_config->wled_port);

    if (app_config->wled_enabled) {
        wled_sync_config_t wled_cfg = {
            .dest_host    = app_config->wled_host,
            .dest_port    = app_config->wled_port,
            .channel_mode = (wled_channel_mode_t)app_config->wled_channel,
        };
        esp_err_t err = wled_sync_init(&wled_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "WLED sync init failed: %s — continuing without it",
                     esp_err_to_name(err));
        }
    }

    raop_config_t config = {
        .device_name     = app_config->device_name,
        .volume_mode     = RAOP_VOLUME_HARDWARE,
        .mdns_mode       = RAOP_MDNS_MANAGED,
        .audio_output_cb = audio_output_callback,
        .event_cb        = raop_event_handler,
        .volume_cb       = raop_volume_callback,
        .pcm_tap_cb      = dsp_pcm_callback,
    };

    esp_err_t err = raop_init(&config, &raop_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start AirPlay: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "AirPlay receiver started: %s", raop_get_device_name(raop_handle));
}

static void dns_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                              const ip_addr_t *addr, u16_t port)
{
    if (p->len < 12) {
        pbuf_free(p);
        return;
    }

    // Simple DNS response - redirect everything to 192.168.4.1
    uint8_t *dns_query = (uint8_t *)p->payload;

    // Build DNS response
    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, p->len + 16, PBUF_RAM);
    if (out) {
        uint8_t *resp = (uint8_t *)out->payload;

        // Copy query header
        memcpy(resp, dns_query, 12);

        // Set response flags
        resp[2] = 0x81; // Response, no error
        resp[3] = 0x80;
        resp[7] = 0x01; // 1 answer

        // Copy question section
        memcpy(resp + 12, dns_query + 12, p->len - 12);

        // Add answer section (point to our IP: 192.168.4.1)
        uint16_t offset = p->len;
        resp[offset++] = 0xc0; // Pointer to name
        resp[offset++] = 0x0c;
        resp[offset++] = 0x00; // Type A
        resp[offset++] = 0x01;
        resp[offset++] = 0x00; // Class IN
        resp[offset++] = 0x01;
        resp[offset++] = 0x00; // TTL
        resp[offset++] = 0x00;
        resp[offset++] = 0x00;
        resp[offset++] = 0x3c;
        resp[offset++] = 0x00; // Data length
        resp[offset++] = 0x04;
        resp[offset++] = 192;  // IP: 192.168.4.1
        resp[offset++] = 168;
        resp[offset++] = 4;
        resp[offset++] = 1;

        udp_sendto(pcb, out, addr, port);
        pbuf_free(out);
    }

    pbuf_free(p);
}

static void start_dns_server(void)
{
    dns_pcb = udp_new();
    if (dns_pcb) {
        udp_bind(dns_pcb, IP_ADDR_ANY, 53);
        udp_recv(dns_pcb, dns_recv_callback, NULL);
        ESP_LOGI(TAG, "DNS server started on port 53");
    }
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "ESP AirPlay Receiver starting...");

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    web_log_init();
    app_config_t app_config = {0};
    config_manager_load(&app_config);

    // Initialize DSP engine (loads presets + active selection from NVS)
    dsp_engine_init();

    // Initialize Stereo Spatial once, after NVS/DSP initialization.
    g_stereo_ctx = stereo_spatial_init(44100);
    if (!g_stereo_ctx) {
        ESP_LOGE(TAG, "Failed to initialize stereo spatial");
    }

    // Initialize I2S output
    i2s_output_init(app_config.bck_pin, app_config.ws_pin, app_config.dout_pin);

    // Initialize WiFi
    ESP_LOGI(TAG, "About to initialize WiFi manager...");
    wifi_manager_init();
    ESP_LOGI(TAG, "WiFi manager initialized");

    // Mark firmware as valid now that core systems are up
    // This confirms the OTA update was successful and prevents rollback
    esp_ota_img_states_t ota_state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "OTA image pending verify, marking valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    if (wifi_manager_is_ap_mode()) {
        start_dns_server();
        ESP_LOGI(TAG, "Captive portal DNS server started (AP mode)");
    } else {
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
        int retry = 0;
        while (!wifi_manager_is_connected() && retry < 240) {
            vTaskDelay(pdMS_TO_TICKS(500));
            retry++;

            /* GOT_IP can arrive while this task sleeps; re-check before logging. */
            if (wifi_manager_is_connected()) {
                break;
            }

            if (retry % 4 == 0) {  // Log every 2 seconds; timeout is 120 seconds.
                ESP_LOGI(TAG, "Still waiting for IP... (%d/120)", retry / 2);
            }
        }

        if (!wifi_manager_is_connected()) {
            ESP_LOGW(TAG, "WiFi not connected after 120 seconds; background reconnect remains active");
        }
    }

    // Start web server for configuration and register all feature routes.
    web_server = web_server_start();

    if (!web_server) {
        ESP_LOGE(TAG, "Web server unavailable; /log and /dsp routes not registered");
    } else {
        // Register log endpoints first so they keep their slots if the URI
        // handler limit is ever reduced accidentally.
        esp_err_t log_err = web_log_register(web_server);
        if (log_err != ESP_OK) {
            ESP_LOGE(TAG, "Web log registration failed: %s", esp_err_to_name(log_err));
        }

        esp_err_t dsp_err = dsp_api_register(web_server);
        if (dsp_err != ESP_OK) {
            ESP_LOGE(TAG, "DSP API registration failed: %s", esp_err_to_name(dsp_err));
        }
    }
    bool airplay_started = false;

    // Wait for WiFi connection (give it some time)
    ESP_LOGI(TAG, "Waiting for network...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Wait for WiFi connection with proper timeout
    if (!wifi_manager_is_ap_mode()) {

        if (wifi_manager_is_connected()) {
            ESP_LOGI(TAG, "Connected! IP: %s", wifi_manager_get_ip());

            // Give network a moment to stabilize
            vTaskDelay(pdMS_TO_TICKS(1000));

            // Now start AirPlay receiver
            start_airplay_receiver(&app_config);
            airplay_started = (raop_handle != NULL);
        } else {
            ESP_LOGW(TAG, "WiFi is not connected yet; reconnect will continue in background");
        }
    }

    ESP_LOGI(TAG, "System ready!");

    if (wifi_manager_is_connected()) {
        ESP_LOGI(TAG, "Connected to WiFi");
        ESP_LOGI(TAG, "IP Address: %s", wifi_manager_get_ip());
        ESP_LOGI(TAG, "Ready to receive AirPlay streams!");
    }

    // Main loop - could add status monitoring here
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        /*
         * A delayed WiFi recovery must still bring AirPlay online. Previously,
         * missing the initial 120-second window left the receiver stopped forever.
         */
        if (!airplay_started && !wifi_manager_is_ap_mode() &&
            wifi_manager_is_connected()) {
            ESP_LOGI(TAG, "WiFi recovered after startup wait; starting AirPlay...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            start_airplay_receiver(&app_config);
            airplay_started = (raop_handle != NULL);
        }

        // Periodic status log
        ESP_LOGI(TAG, "Status: WiFi=%s, IP=%s",
                 wifi_manager_is_connected() ? "Connected" : "Disconnected",
                 wifi_manager_get_ip());
    }
}