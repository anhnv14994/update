idf_component_register(
    SRCS "spdif_in.c" "histogram.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    PRIV_REQUIRES driver freertos esp_timer heap
)
