#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "wifi_manager";

#define WIFI_SSID_KEY "wifi_ssid"
#define WIFI_PASS_KEY "wifi_pass"
#define WIFI_NAMESPACE "wifi_config"

#define AP_SSID_PREFIX "ESP-AIRSYNC-"
#define AP_PASSWORD "espairsync"

static bool s_wifi_connected = false;
static char s_ip_address[16] = "";
static char s_saved_ssid[32] = "";
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_ap_mode = false;
static esp_timer_handle_t s_reconnect_timer = NULL;
static uint32_t s_reconnect_attempt = 0;

/* Reconnect outside the default event-loop task. */
static void reconnect_timer_cb(void *arg)
{
    (void)arg;
    if (s_ap_mode || s_wifi_connected) return;

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN) {
        ESP_LOGW(TAG, "WiFi reconnect request failed: %s", esp_err_to_name(err));
    }
}

static void schedule_reconnect(void)
{
    if (!s_reconnect_timer || s_ap_mode || s_wifi_connected) return;

    uint32_t attempt = s_reconnect_attempt;
    if (attempt > 4) attempt = 4;
    if (s_reconnect_attempt < 4) s_reconnect_attempt++;

    /* 500 ms, 1 s, 2 s, 4 s, then 8 s. */
    uint64_t delay_us = (500ULL << attempt) * 1000ULL;
    (void)esp_timer_stop(s_reconnect_timer);

    esp_err_t err = esp_timer_start_once(s_reconnect_timer, delay_us);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to schedule WiFi reconnect: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGW(TAG, "WiFi reconnect scheduled in %llu ms (attempt %u)",
             (unsigned long long)(delay_us / 1000ULL),
             (unsigned)s_reconnect_attempt);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started, connecting...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Initial WiFi connect request failed: %s", esp_err_to_name(err));
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP; waiting for DHCP");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event =
            (wifi_event_sta_disconnected_t *)event_data;
        int reason = event ? event->reason : -1;

        ESP_LOGW(TAG, "WiFi disconnected! reason=%d, ssid=%s",
                 reason, s_saved_ssid);

        s_wifi_connected = false;
        s_ip_address[0] = '\0';
        schedule_reconnect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        if (event) {
            snprintf(s_ip_address, sizeof(s_ip_address), IPSTR, IP2STR(&event->ip_info.ip));
            s_wifi_connected = (event->ip_info.ip.addr != 0);
            if (s_wifi_connected) {
                s_reconnect_attempt = 0;
                if (s_reconnect_timer) (void)esp_timer_stop(s_reconnect_timer);
                ESP_LOGI(TAG, "Got IP: %s", s_ip_address);
            }
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        if (event) {
            ESP_LOGI(TAG, "Station %02X:%02X:%02X:%02X:%02X:%02X joined AP, AID=%d",
                     event->mac[0], event->mac[1], event->mac[2],
                     event->mac[3], event->mac[4], event->mac[5], event->aid);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        if (event) {
            ESP_LOGI(TAG, "Station %02X:%02X:%02X:%02X:%02X:%02X left AP, AID=%d, reason=%d",
                     event->mac[0], event->mac[1], event->mac[2],
                     event->mac[3], event->mac[4], event->mac[5],
                     event->aid, event->reason);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP started successfully");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*) event_data;
        if (event) {
            ESP_LOGI(TAG, "DHCP assigned IP " IPSTR " to station", IP2STR(&event->ip));
        }
    }
}

static bool load_wifi_credentials(char *ssid, char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No saved WiFi credentials found: %s", esp_err_to_name(err));
        return false;
    }

    size_t ssid_len = 32;
    size_t pass_len = 64;

    err = nvs_get_str(nvs_handle, WIFI_SSID_KEY, ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SSID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_get_str(nvs_handle, WIFI_PASS_KEY, password, &pass_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get password: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Loaded WiFi credentials:");
    ESP_LOGI(TAG, "  SSID: '%s' (len=%d)", ssid, strlen(ssid));

    return true;
}

static void start_ap_mode(void)
{
    uint8_t mac[6];
    char ap_ssid[32];

    esp_wifi_get_mac(WIFI_IF_AP, mac);
    snprintf(ap_ssid, sizeof(ap_ssid), "%s%02X%02X%02X",
             AP_SSID_PREFIX, mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = 0,
            .channel = 3,
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
            // .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pairwise_cipher = WIFI_CIPHER_TYPE_NONE,
            .beacon_interval = 100,
            .pmf_cfg = {
                .required = false,
                .capable = false,
            },
        },
    };

    // Copy SSID
    strncpy((char *)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ap_ssid);

    // Copy password explicitly
    // strncpy((char *)wifi_config.ap.password, AP_PASSWORD, sizeof(wifi_config.ap.password) - 1);

    // Set WiFi country code
    wifi_country_t country = {
        .cc = "US",
        .schan = 1,
        .nchan = 11,
        .policy = WIFI_COUNTRY_POLICY_AUTO,
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_LOGI(TAG, "Starting AP with SSID: '%s' (len=%d), Password: (len=%d)",
             (char*)wifi_config.ap.ssid, wifi_config.ap.ssid_len,
             (char*)wifi_config.ap.password, strlen((char*)wifi_config.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE); // AirPlay realtime, tránh jitter do modem sleep

    s_ap_mode = true;

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Started in AP mode, connect to WiFi:");
    ESP_LOGI(TAG, "  SSID: %s", ap_ssid);
    ESP_LOGI(TAG, "  Config page: http://192.168.4.1");
    ESP_LOGI(TAG, "===========================================");
}

void wifi_manager_init(void)
{
    ESP_LOGI(TAG, "=== WiFi Manager Init START ===");

    esp_err_t ret = nvs_flash_init();
    ESP_LOGI(TAG, "NVS init result: %d", ret);
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing TCP/IP stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "Creating default event loop...");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Initializing WiFi with default config...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_LOGI(TAG, "Clearing any stale WiFi associations...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    vTaskDelay(pdMS_TO_TICKS(100));

    char ssid[32] = {0};
    char password[64] = {0};
    ESP_LOGI(TAG, "Checking for saved WiFi credentials...");
    bool have_credentials = load_wifi_credentials(ssid, password);

    /*
     * The default netif installs ID-specific ESP-IDF handlers. Create it
     * first, then register our own ID-specific handlers below. Do not use
     * ESP_EVENT_ANY_ID here: base-level handlers run before ID-level handlers.
     */
    if (have_credentials) {
        s_ap_mode = false;
        strlcpy(s_saved_ssid, ssid, sizeof(s_saved_ssid));
        s_sta_netif = esp_netif_create_default_wifi_sta();

        esp_timer_create_args_t timer_args = {
            .callback = reconnect_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wifi_reconnect",
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_reconnect_timer));
    } else {
        s_ap_mode = true;
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    ESP_LOGI(TAG, "Registering application event handlers...");
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_STA_START, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_AP_START, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &wifi_event_handler, NULL));

    if (!have_credentials) {
        ESP_LOGI(TAG, "No credentials found, starting AP mode");
        start_ap_mode();
    } else {
        ESP_LOGI(TAG, "Credentials found, connecting to WiFi: %s", ssid);

        wifi_config_t wifi_config = {0};
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        /* Scan all channels to tolerate AP channel changes. */
        wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    }

    ESP_LOGI(TAG, "=== WiFi Manager Init COMPLETE ===");
}
bool wifi_manager_save_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    ESP_LOGI(TAG, "Saving credentials:");
    ESP_LOGI(TAG, "  SSID: '%s' (len=%d)", ssid, strlen(ssid));
    ESP_LOGI(TAG, "  Pass: '%s' (len=%d)", password, strlen(password));

    err = nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_str(nvs_handle, WIFI_SSID_KEY, ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SSID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_set_str(nvs_handle, WIFI_PASS_KEY, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set password: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials saved successfully");
        return true;
    }

    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    return false;
}

const char* wifi_manager_get_saved_ssid(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return s_saved_ssid; // returns ""
    }

    size_t len = sizeof(s_saved_ssid);
    nvs_get_str(nvs_handle, WIFI_SSID_KEY, s_saved_ssid, &len);
    nvs_close(nvs_handle);

    return s_saved_ssid;
}

bool wifi_manager_is_connected(void)
{
    return s_wifi_connected;
}

const char* wifi_manager_get_ip(void)
{
    return s_ip_address;
}

bool wifi_manager_is_ap_mode(void)
{
    return s_ap_mode;
}