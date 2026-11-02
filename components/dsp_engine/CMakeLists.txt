idf_component_register(
    SRCS "dsp_engine.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_timer
        esp_hw_support
)
