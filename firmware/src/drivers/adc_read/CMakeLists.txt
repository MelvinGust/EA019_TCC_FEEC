idf_component_register(
    SRCS "adc_read.c"
    INCLUDE_DIRS "."
    REQUIRES
    PRIV_REQUIRES "esp_timer" "esp_adc"
)   