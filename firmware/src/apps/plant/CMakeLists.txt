idf_component_register(
    SRCS "plant.c"
    INCLUDE_DIRS "."
    REQUIRES
    PRIV_REQUIRES "utils" "esp_driver_uart" "esp_driver_gpio" "lwip" "esp_timer" "mqtt" "nvs_flash" "esp_netif" "protocol_examples_common"
)
