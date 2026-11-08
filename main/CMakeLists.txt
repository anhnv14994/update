idf_component_register(
    SRCS "main.c" "wifi_manager.c" "web_server.c" "config_manager.c" "dsp_api.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "web_config.html" "web_update.html" "web_style.css" "web_dsp.html"
    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_timer
        esp_http_server
        driver
        wifi_provisioning
        i2s_output
        spdif_input
        esp-raop-receiver
        wled_sync
        dsp_engine
        stereo_spatial
        web_log
        json
        esp_system
        app_update
)