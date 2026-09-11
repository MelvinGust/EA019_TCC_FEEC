idf_component_register(
    SRCS "controller.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "utils" "esp_driver_uart" "esp_driver_gpio" "esp_timer" "lwip"
)