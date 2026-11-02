idf_component_register(
    SRCS "web_log.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_http_server
        heap
)
